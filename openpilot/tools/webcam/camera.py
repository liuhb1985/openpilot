import av
import cv2 as cv

# Camera resolutions the compiled modeld warp JITs are built for
# (see selfdrive/modeld/compile_modeld.py --camera-resolutions). Frames are
# resized to the closest one when the capture device cannot provide them.
SUPPORTED_CAM_RESOLUTIONS = ((1928, 1208), (1344, 760))

class Camera:
  def __init__(self, cam_type_state, stream_type, camera_id, formats=None):
    try:
      camera_id = int(camera_id)
    except ValueError: # allow strings, ex: /dev/video0
      pass
    self.cam_type_state = cam_type_state
    self.stream_type = stream_type
    self.cur_frame_id = 0

    print(f"Opening {cam_type_state} at {camera_id}")

    self.cap = self._open_capture(camera_id, formats)

    cap_w = int(self.cap.get(cv.CAP_PROP_FRAME_WIDTH))
    cap_h = int(self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))

    # modeld needs a resolution with a compiled warp; rescale in software
    # to the supported resolution with the closest aspect ratio.
    if (cap_w, cap_h) in SUPPORTED_CAM_RESOLUTIONS:
      self.W, self.H = cap_w, cap_h
    else:
      self.W, self.H = min(SUPPORTED_CAM_RESOLUTIONS,
                           key=lambda r: abs(r[0] / r[1] - cap_w / cap_h))
      print(f"Resizing {cam_type_state} from {cap_w}x{cap_h} to {self.W}x{self.H} "
            f"(no compiled model warp for {cap_w}x{cap_h})")

  @staticmethod
  def _open_capture(camera_id, formats=None):
    # Both USB cameras usually share one USB 2.0 bus (~53 MB/s). An
    # uncompressed YUYV 720p stream alone uses ~44 MB/s and starves the other
    # camera, so each camera tries its format list in order and the first one
    # that actually delivers a frame wins.
    if not formats:
      formats = (('MJPG', 1280, 720), (None, 1280, 720))
    for fourcc, w, h in formats:
      cap = cv.VideoCapture(camera_id)
      if fourcc:
        cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*fourcc))
      if w:
        cap.set(cv.CAP_PROP_FRAME_WIDTH, float(w))
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, float(h))
      cap.set(cv.CAP_PROP_FPS, 25.0)
      if cap.grab():
        fmt = int(cap.get(cv.CAP_PROP_FOURCC))
        fmt_name = ''.join(chr((fmt >> 8 * i) & 0xFF) for i in range(4))
        print(f"  capture format: {fmt_name} "
              f"{int(cap.get(cv.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))}")
        return cap
      cap.release()
    raise OSError(f"cannot read frames from camera {camera_id}")

  @classmethod
  def bgr2nv12(self, bgr):
    frame = av.VideoFrame.from_ndarray(bgr, format='bgr24')
    return frame.reformat(format='nv12').to_ndarray()

  def read_frames(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      # Rotate the frame 180 degrees (flip both axes)
      frame = cv.flip(frame, -1)
      if frame.shape[1] != self.W or frame.shape[0] != self.H:
        frame = cv.resize(frame, (self.W, self.H))
      yuv = Camera.bgr2nv12(frame)
      yield yuv.data.tobytes()
    self.cap.release()
