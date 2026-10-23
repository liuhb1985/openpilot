import os
import platform
import queue
import threading
import time

import av
import cv2 as cv

# webcamerad runs one Python thread per camera, so all decode/resize work already
# serializes on the GIL. Left to its defaults OpenCV also spins up its own thread
# pool and may grab the GPU through OpenCL, which fights the driving model's
# tinygrad GPU work and adds scheduling jitter that shows up as camera/model rate
# drops. (mirrors the dst project)
cv.setUseOpenCL(False)
cv.setNumThreads(1)

# USB capture devices are enumerated asynchronously on Linux; give the camera a
# short grace period to show up instead of dying for good on the first attempt.
OPEN_TIMEOUT_S = float(os.getenv("WEBCAM_OPEN_TIMEOUT") or "10")

# Camera rate used by both the V4L2 negotiation and the paced capture loop.
CAMERA_FPS = int(os.getenv("WEBCAM_FPS") or 20)

# Some mounts (webcam screwed upside down into the bracket) need a 180° rotation.
# On a bench the camera stands upright, so default to no rotation: the flip also
# costs a full frame copy on every single frame.
ROTATE_180 = os.getenv("WEBCAM_ROTATE180", "0") == "1"

# Capture resolution. Always 1920x1080: both webcams negotiate it, and it gives
# the driving model the sharpest available input. Override only for hardware
# that cannot sustain 1080p.
CAPTURE_WIDTH = int(os.getenv("WEBCAM_WIDTH") or 1920)
CAPTURE_HEIGHT = int(os.getenv("WEBCAM_HEIGHT") or 1080)

# Camera resolutions the compiled modeld warp JITs are built for
# (see selfdrive/modeld/compile_modeld.py --camera-resolutions). Every camera
# must publish the SAME resolution: modeld derives its NV12 geometry for both
# 'img' and 'big_img' from the road camera size (selfdrive/modeld/modeld.py).
# 1920x1080 capture is rescaled only when the loaded model has no warp for it.
# Keep in sync with openpilot/common/transformations/camera.py, which drives the
# resolutions modeld compiles (modeld/SConscript CAMERA_CONFIGS).
SUPPORTED_CAM_RESOLUTIONS = ((1920, 1080), (1344, 760))

# Optional environment override for the capture pixel format (e.g. YUYV).
FOURCC_OVERRIDE = os.getenv("WEBCAM_FOURCC")

# Bounded hand-off between the capture thread and the publish loop. The capture
# thread reads/decodes as fast as the camera delivers; the publish loop drains at
# exactly CAMERA_FPS. Without this split a single slow decode - or GIL contention
# from the other camera's thread - slows the *published* rate, and selfdrived's
# all_freq_ok then raises cameraFrameRate. When the queue is full the oldest frame
# is dropped, which bounds the added latency to CAMERA_QUEUE_SIZE frames.
# (ported from the dst project's CameraMJPG._frame_pipeline)
CAMERA_QUEUE_SIZE = max(1, int(os.getenv("WEBCAM_QUEUE_SIZE") or "3"))

# V4L2 records the real exposure time per buffer and OpenCV exposes it as
# CAP_PROP_POS_MSEC, in milliseconds of CLOCK_MONOTONIC - the same domain as
# time.monotonic_ns(), and a few ms more accurate than the read() time because it
# excludes driver buffering. Some cameras/backends return 0 or a value from a
# different epoch though, so it is only trusted inside this window.
POS_MSEC_MAX_SKEW_S = 5.0

# Auto exposure trades frame rate for brightness: in the dark it stretches the
# exposure time and the camera drops to ~13-18 fps. A fixed exposure can keep
# ~28 fps, but a value tuned in daylight leaves the image almost black at night,
# and some UVC cameras silently ignore manual exposure/gain in MJPEG mode.
# Everything below is therefore opt-in, and a too-dark manual setting falls back
# to auto exposure automatically.
def _env_for(cam_type_state: str, suffix: str):
  role = "ROAD" if cam_type_state == "roadCameraState" else ("WIDE" if cam_type_state == "wideRoadCameraState" else "DRIVER")
  return os.getenv(f"WEBCAM_{suffix}_{role}") or os.getenv(f"WEBCAM_{suffix}")

AUTO_EXPOSURE_AUTO = 3.0
AUTO_EXPOSURE_MANUAL = 1.0
# Below this average luma a frame is considered "black"; auto exposure is
# restored when a manual setting produces it.
BRIGHTNESS_FLOOR = float(os.getenv("WEBCAM_MIN_BRIGHTNESS") or 20.0)


def pick_output_resolution(cap_w: int, cap_h: int) -> tuple[int, int]:
  """Return the published resolution shared by every camera.

  Uses the capture size when the model has a compiled warp for it, otherwise the
  supported resolution with the closest aspect ratio (frames are rescaled).
  """
  if (cap_w, cap_h) in SUPPORTED_CAM_RESOLUTIONS:
    return cap_w, cap_h
  return min(SUPPORTED_CAM_RESOLUTIONS, key=lambda r: abs(r[0] / r[1] - cap_w / cap_h))


class Camera:
  def __init__(self, cam_type_state, stream_type, camera_id, formats=None):
    try:
      camera_id = int(camera_id)
    except ValueError: # allow strings, ex: /dev/video0 or /dev/v4l/by-id/...
      pass
    self.cam_type_state = cam_type_state
    self.stream_type = stream_type
    self.cur_frame_id = 0
    self.rotate_180 = ROTATE_180

    print(f"Opening {cam_type_state} at {camera_id}")

    self.cap = self._open_capture(camera_id, formats)

    self.cap_w = int(self.cap.get(cv.CAP_PROP_FRAME_WIDTH))
    self.cap_h = int(self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    if self.cap_w <= 0 or self.cap_h <= 0:
      raise RuntimeError(f"{cam_type_state} camera at {camera_id} reports an invalid "
                         f"{self.cap_w}x{self.cap_h} resolution")

    # Published size; camerad aligns every camera on the road camera's size.
    self.W, self.H = self.cap_w, self.cap_h

    self._apply_image_controls()

    # The capture thread is started lazily by read_frames(): camerad calls
    # set_output_size() after construction, and that must not race with the reader.
    self._queue: queue.Queue[tuple[bytes, int]] | None = None
    self._stop_event: threading.Event | None = None
    self._read_error: BaseException | None = None
    self._read_thread: threading.Thread | None = None

  def _frame_timestamp(self) -> int:
    """Monotonic ns for the frame that was just read.

    Prefers the driver's own per-frame timestamp (the true exposure time) when the
    camera reports a sane one, otherwise the moment the frame landed in user space.
    Either way it is taken before decode/resize/conversion so the two
    unsynchronized USB cameras only differ by their delivery jitter.
    """
    now_ns = time.monotonic_ns()
    try:
      pos_ms = self.cap.get(cv.CAP_PROP_POS_MSEC)
    except Exception:
      return now_ns
    if not pos_ms or pos_ms <= 0:
      return now_ns
    pos_ns = int(pos_ms * 1e6)
    # Reject values from a different clock epoch (or garbage) and fall back.
    return pos_ns if abs(pos_ns - now_ns) < POS_MSEC_MAX_SKEW_S * 1e9 else now_ns

  def _read_loop(self, q: queue.Queue[tuple[bytes, int]], stop_event: threading.Event) -> None:
    """Capture thread: read, timestamp, convert, hand off. Never waits on publish."""
    while not stop_event.is_set():
      try:
        ret, frame = self.cap.read()
      except Exception as e:
        self._read_error = e
        break
      if not ret:
        self._read_error = RuntimeError(f"{self.cam_type_state} camera stopped delivering frames")
        break

      ts = self._frame_timestamp()
      if self.rotate_180:
        frame = cv.flip(frame, -1)  # upside-down mount
      if frame.shape[1] != self.W or frame.shape[0] != self.H:
        frame = cv.resize(frame, (self.W, self.H))
      yuv = Camera.bgr2nv12(frame)

      if q.full():
        # Drop the oldest so the newest frame always gets in; this bounds the
        # capture -> publish latency to CAMERA_QUEUE_SIZE frames.
        try:
          q.get_nowait()
        except queue.Empty:
          pass
      q.put((yuv.data.tobytes(), ts))

  def _apply_image_controls(self) -> None:
    """Exposure/gain are optional. A manual setting that leaves the image black
    (night, or a camera that ignores manual controls) falls back to auto."""
    exposure = _env_for(self.cam_type_state, "EXPOSURE")
    gain = _env_for(self.cam_type_state, "GAIN")
    if exposure is None and gain is None:
      return

    mode = _env_for(self.cam_type_state, "AUTO_EXPOSURE")
    try:
      self.cap.set(cv.CAP_PROP_AUTO_EXPOSURE, float(mode) if mode else
                   (AUTO_EXPOSURE_MANUAL if exposure is not None else AUTO_EXPOSURE_AUTO))
      if exposure is not None:
        self.cap.set(cv.CAP_PROP_EXPOSURE, float(exposure))
      if gain is not None:
        self.cap.set(cv.CAP_PROP_GAIN, float(gain))
    except Exception:
      print(f"Warning: couldn't set exposure/gain for {self.cam_type_state}")
      return

    # Let the sensor settle, then verify the result instead of trusting it.
    time.sleep(0.5)
    brightness = self._measure_brightness()
    if brightness is None:
      return
    print(f"  {self.cam_type_state}: exposure={exposure or 'auto'} gain={gain or 'default'} "
          f"-> luma {brightness:.1f}")
    if exposure is not None and brightness < BRIGHTNESS_FLOOR:
      print(f"  {self.cam_type_state}: manual exposure leaves the image almost black "
            f"(luma {brightness:.1f} < {BRIGHTNESS_FLOOR}); switching back to auto exposure")
      try:
        self.cap.set(cv.CAP_PROP_AUTO_EXPOSURE, AUTO_EXPOSURE_AUTO)
      except Exception:
        pass
      time.sleep(0.5)
      brightness = self._measure_brightness()
      if brightness is not None:
        print(f"  {self.cam_type_state}: auto exposure -> luma {brightness:.1f}")

  def _measure_brightness(self):
    """Average luma of the next frame, or None if no frame arrives."""
    for _ in range(10):
      ret, frame = self.cap.read()
      if ret and frame is not None and frame.size:
        return float(cv.cvtColor(frame, cv.COLOR_BGR2GRAY).mean())
      time.sleep(0.05)
    return None

  def set_output_size(self, width: int, height: int) -> None:
    if (width, height) != (self.cap_w, self.cap_h):
      print(f"Resizing {self.cam_type_state} from {self.cap_w}x{self.cap_h} to {width}x{height} "
            f"(no compiled model warp for {self.cap_w}x{self.cap_h})")
    self.W, self.H = width, height

  @staticmethod
  def _open_capture(camera_id, formats=None):
    # Both USB cameras usually share one USB 2.0 bus (~53 MB/s). An uncompressed
    # YUYV 720p stream alone uses ~44 MB/s and starves the second camera, so
    # each camera walks its format list and keeps the first one that really
    # delivers a frame. Order matters: the road camera asks for full quality
    # first, secondary cameras try a bandwidth-friendly format first.
    if not formats:
      # MJPEG is mandatory here: uncompressed 1080p (~79 MB/s) alone exceeds the
      # ~53 MB/s of a USB 2.0 bus shared by both cameras.
      formats = (('MJPG', CAPTURE_WIDTH, CAPTURE_HEIGHT), (None, CAPTURE_WIDTH, CAPTURE_HEIGHT))
    if FOURCC_OVERRIDE:
      formats = tuple((FOURCC_OVERRIDE, w, h) for _fourcc, w, h in formats)

    deadline = time.monotonic() + OPEN_TIMEOUT_S
    last_error = None
    while True:
      for fourcc, w, h in formats:
        # On Linux ask for the V4L2 backend explicitly. The launcher binds cameras
        # by USB serial (/dev/v4l/by-id/usb-...-video-index0) and OpenCV otherwise
        # picks FFmpeg/GStreamer for a full path, which treats the device as a
        # media file and silently ignores the FOURCC/FPS negotiation below.
        # (mirrors the dst project)
        if platform.system() == "Linux":
          cap = cv.VideoCapture(camera_id, cv.CAP_V4L2)
        else:
          cap = cv.VideoCapture(camera_id)
        if not cap.isOpened():
          cap.release()
          continue
        if fourcc:
          try:
            cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*fourcc[:4]))
          except Exception:
            print(f"Warning: couldn't set fourcc {fourcc} for {camera_id}")
        if w:
          cap.set(cv.CAP_PROP_FRAME_WIDTH, float(w))
          cap.set(cv.CAP_PROP_FRAME_HEIGHT, float(h))
        cap.set(cv.CAP_PROP_FPS, float(CAMERA_FPS))
        # grab() proves the negotiated format really streams; a second camera
        # on a saturated USB bus opens fine but never delivers a frame.
        if cap.grab():
          fmt = int(cap.get(cv.CAP_PROP_FOURCC))
          fmt_name = ''.join(chr((fmt >> 8 * i) & 0xFF) for i in range(4))
          print(f"Opened {camera_id}: {fmt_name} "
                f"{int(cap.get(cv.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))} "
                f"@ {cap.get(cv.CAP_PROP_FPS):.1f} fps")
          return cap
        cap.release()
        last_error = f"no frames from {camera_id} at {fourcc or 'default'} {w}x{h}"
      if time.monotonic() >= deadline:
        break
      time.sleep(0.5)
    raise RuntimeError(f"failed to open {camera_id}: {last_error}")

  @classmethod
  def bgr2nv12(self, bgr):
    frame = av.VideoFrame.from_ndarray(bgr, format='bgr24')
    return frame.reformat(format='nv12').to_ndarray()

  def read_frames(self):
    q, stop_event, read_thread = self._queue, self._stop_event, self._read_thread
    if read_thread is None or q is None or stop_event is None:
      # Lazy start: camerad calls set_output_size() after construction.
      q = queue.Queue(maxsize=CAMERA_QUEUE_SIZE)
      stop_event = threading.Event()
      self._queue, self._stop_event = q, stop_event
      self._read_error = None
      read_thread = threading.Thread(target=self._read_loop, args=(q, stop_event),
                                     name=f"cam-{self.cam_type_state}", daemon=True)
      self._read_thread = read_thread
      read_thread.start()

    try:
      # Drain at the caller's pace (camerad's Ratekeeper), so the published rate
      # is CAMERA_FPS regardless of how long decode takes. A sustained decode
      # overload still starves this loop - the queue only absorbs jitter.
      while not stop_event.is_set() or not q.empty():
        try:
          yield q.get(timeout=0.5)
        except queue.Empty:
          if stop_event.is_set():
            break
    finally:
      stop_event.set()
      if read_thread.is_alive():
        read_thread.join(timeout=2.0)
      self.cap.release()
    # A silent return here used to leave webcamerad alive publishing nothing,
    # which looks identical to "UI shows no camera feed". Fail loudly instead.
    if self._read_error is not None:
      raise self._read_error
    raise RuntimeError(f"{self.cam_type_state} camera stopped delivering frames")
