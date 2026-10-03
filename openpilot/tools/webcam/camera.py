import os
import platform
import time

import av
import cv2 as cv

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
# 1920x1080 capture is rescaled to whichever of these is available.
SUPPORTED_CAM_RESOLUTIONS = ((1928, 1208), (1344, 760))

# Optional environment override for the capture pixel format (e.g. YUYV).
FOURCC_OVERRIDE = os.getenv("WEBCAM_FOURCC")

# Auto exposure trades frame rate for brightness: in a dim room it stretches the
# exposure time and the camera drops to ~18 fps (and ~12 fps per camera once
# both share the bus). Setting a fixed exposure restores ~28 fps, but it cannot
# adapt to tunnel/night lighting, so it stays opt-in for bench use.
EXPOSURE_OVERRIDE = os.getenv("WEBCAM_EXPOSURE")
AUTO_EXPOSURE_MODE = os.getenv("WEBCAM_AUTO_EXPOSURE")


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
          if EXPOSURE_OVERRIDE is not None:
            try:
              cap.set(cv.CAP_PROP_AUTO_EXPOSURE, float(AUTO_EXPOSURE_MODE or 1.0))
              cap.set(cv.CAP_PROP_EXPOSURE, float(EXPOSURE_OVERRIDE))
            except Exception:
              print(f"Warning: couldn't set exposure {EXPOSURE_OVERRIDE} for {camera_id}")
          fmt = int(cap.get(cv.CAP_PROP_FOURCC))
          fmt_name = ''.join(chr((fmt >> 8 * i) & 0xFF) for i in range(4))
          print(f"Opened {camera_id}: {fmt_name} "
                f"{int(cap.get(cv.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))} "
                f"@ {cap.get(cv.CAP_PROP_FPS):.1f} fps"
                + (f", exposure {EXPOSURE_OVERRIDE}" if EXPOSURE_OVERRIDE is not None else ""))
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
    try:
      while True:
        ret, frame = self.cap.read()
        if not ret:
          break
        if self.rotate_180:
          frame = cv.flip(frame, -1)  # upside-down mount
        if frame.shape[1] != self.W or frame.shape[0] != self.H:
          frame = cv.resize(frame, (self.W, self.H))
        yuv = Camera.bgr2nv12(frame)
        yield yuv.data.tobytes()
    finally:
      self.cap.release()
    # A silent return here used to leave webcamerad alive publishing nothing,
    # which looks identical to "UI shows no camera feed". Fail loudly instead.
    raise RuntimeError(f"{self.cam_type_state} camera stopped delivering frames")
