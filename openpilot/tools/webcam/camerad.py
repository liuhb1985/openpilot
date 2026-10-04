#!/usr/bin/env python3
import threading
import os
import platform
import time
from collections import namedtuple

from msgq.visionipc import VisionIpcServer, VisionStreamType
from openpilot.cereal import messaging

from openpilot.tools.webcam.camera import Camera, CAMERA_FPS, pick_output_resolution
from openpilot.common.realtime import Ratekeeper

ROAD_CAM = os.getenv("ROAD_CAM", "0")
WIDE_CAM = os.getenv("WIDE_CAM")
DRIVER_CAM = os.getenv("DRIVER_CAM")

CameraType = namedtuple("CameraType", ["msg_name", "stream_type", "cam_id", "formats"])


def resolve_camera_device(cam_id: str):
  """Turn a ROAD_CAM/WIDE_CAM/DRIVER_CAM value into something OpenCV can open.

  Accepted values:
    - plain index:  "0" -> /dev/video0 (documented in README)
    - full path:    "/dev/video2", "/dev/v4l/by-id/usb-...-video-index0"
  The PC launcher binds cameras through /dev/v4l/by-id so a USB port change can't
  reshuffle indices, so we must not blindly prefix those with /dev/video.
  """
  # on macOS, AVFoundation wants the index itself
  if platform.system() == "Darwin":
    return cam_id
  return cam_id if str(cam_id).startswith("/") else f"/dev/video{cam_id}"


# Both cameras capture 1920x1080, but they share one USB 2.0 bus (~53 MB/s)
# where uncompressed 1080p (~79 MB/s) cannot fit, so MJPEG comes first for every
# camera. Tried in order; the first format that actually delivers a frame wins.
CAPTURE_SIZE = (1920, 1080)
ROAD_CAM_FORMATS = (('MJPG', *CAPTURE_SIZE), (None, *CAPTURE_SIZE))
SECONDARY_CAM_FORMATS = ROAD_CAM_FORMATS

CAMERAS = [
  CameraType("roadCameraState", VisionStreamType.VISION_STREAM_ROAD, ROAD_CAM, ROAD_CAM_FORMATS),
]
if WIDE_CAM:
  CAMERAS.append(CameraType("wideRoadCameraState", VisionStreamType.VISION_STREAM_WIDE_ROAD, WIDE_CAM, SECONDARY_CAM_FORMATS))
if DRIVER_CAM:
  CAMERAS.append(CameraType("driverCameraState", VisionStreamType.VISION_STREAM_DRIVER, DRIVER_CAM, SECONDARY_CAM_FORMATS))

class Camerad:
  def __init__(self):
    self.pm = messaging.PubMaster([c.msg_name for c in CAMERAS])
    self.vipc_server = VisionIpcServer("camerad")

    self.cameras = []
    for c in CAMERAS:
      cam = Camera(c.msg_name, c.stream_type, resolve_camera_device(c.cam_id), c.formats)
      self.cameras.append(cam)

    # modeld derives NV12 geometry for both 'img' and 'big_img' from the road
    # camera size, so every stream is published at that same resolution.
    out_w, out_h = pick_output_resolution(self.cameras[0].cap_w, self.cameras[0].cap_h)

    for cam in self.cameras:
      cam.set_output_size(out_w, out_h)
      self.vipc_server.create_buffers(cam.stream_type, 20, cam.W, cam.H)

    self.vipc_server.start_listener()

  def _send_yuv(self, yuv, frame_id, pub_type, yuv_type):
    # Timestamps must be a clock shared by every camera. Deriving them from the
    # per-camera frame counter made the road and wide streams drift apart (the
    # camera that started first stays seconds ahead forever), and modeld's frame
    # sync loop waits for the road frame to lead the wide one by 25 ms -- so it
    # never ran and no modelV2 was published.
    ts = time.time_ns()
    self.vipc_server.send(yuv_type, yuv, frame_id, ts, ts)
    dat = messaging.new_message(pub_type, valid=True)
    msg = {
      "frameId": frame_id,
      "transform": [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
    }
    setattr(dat, pub_type, msg)
    self.pm.send(pub_type, dat)

  def camera_runner(self, cam):
    rk = Ratekeeper(CAMERA_FPS, None)
    for yuv in cam.read_frames():
      self._send_yuv(yuv, cam.cur_frame_id, cam.cam_type_state, cam.stream_type)
      cam.cur_frame_id += 1
      rk.keep_time()

  def run(self):
    threads = []
    for cam in self.cameras:
      cam_thread = threading.Thread(target=self.camera_runner, args=(cam,))
      cam_thread.start()
      threads.append(cam_thread)

    for t in threads:
      t.join()


def main():
  camerad = Camerad()
  camerad.run()


if __name__ == "__main__":
  main()
