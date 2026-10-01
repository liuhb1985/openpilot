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


class Camera:
  def __init__(self, cam_type_state, stream_type, camera_id):
    try:
      camera_id = int(camera_id)
    except ValueError: # allow strings, ex: /dev/video0 or /dev/v4l/by-id/...
      pass
    self.cam_type_state = cam_type_state
    self.stream_type = stream_type
    self.cur_frame_id = 0
    self.rotate_180 = ROTATE_180

    print(f"Opening {cam_type_state} at {camera_id}")

    self.cap = cv.VideoCapture(camera_id)
    deadline = time.monotonic() + OPEN_TIMEOUT_S
    while not self.cap.isOpened() and time.monotonic() < deadline:
      time.sleep(0.5)
      self.cap.release()
      self.cap.open(camera_id)
    if not self.cap.isOpened():
      raise RuntimeError(f"failed to open {cam_type_state} camera at {camera_id}; "
                         "check that the device exists and is readable by this user")

    # V4L2 leaves most USB webcams on YUYV, which caps 720p at single digit fps and
    # looks exactly like "very choppy video". Ask for MJPEG first (override with
    # WEBCAM_FOURCC=YUYV ...), then negotiate resolution/rate.
    fourcc = os.getenv("WEBCAM_FOURCC", "MJPG" if platform.system() != "Darwin" else "")
    if fourcc:
      try:
        self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*fourcc[:4]))
      except Exception:
        print(f"Warning: couldn't set fourcc {fourcc} for {cam_type_state}")

    self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280.0)
    self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720.0)
    self.cap.set(cv.CAP_PROP_FPS, float(CAMERA_FPS))

    self.W = int(self.cap.get(cv.CAP_PROP_FRAME_WIDTH))
    self.H = int(self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    if self.W <= 0 or self.H <= 0:
      raise RuntimeError(f"{cam_type_state} camera at {camera_id} reports an invalid {self.W}x{self.H} resolution")

    print(f"Opened {cam_type_state} at {camera_id}: {self.W}x{self.H} @ {self.cap.get(cv.CAP_PROP_FPS):.1f} fps")

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
        yuv = Camera.bgr2nv12(frame)
        yield yuv.data.tobytes()
    finally:
      self.cap.release()
    # A silent return here used to leave webcamerad alive publishing nothing,
    # which looks identical to "UI shows no camera feed". Fail loudly instead.
    raise RuntimeError(f"{self.cam_type_state} camera stopped delivering frames")
