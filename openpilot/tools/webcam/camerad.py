#!/usr/bin/env python3
import threading
import os
import platform
from collections import namedtuple

from msgq.visionipc import VisionIpcServer, VisionStreamType
from openpilot.cereal import messaging

from openpilot.tools.webcam.camera import Camera
from openpilot.common.realtime import Ratekeeper

ROAD_CAM = os.getenv("ROAD_CAM", "0")
WIDE_CAM = os.getenv("WIDE_CAM")
DRIVER_CAM = os.getenv("DRIVER_CAM")

CameraType = namedtuple("CameraType", ["msg_name", "stream_type", "cam_id", "formats"])

# Cameras usually share one USB 2.0 bus (~53 MB/s). The road camera feeds the
# driving model, so it gets the full-quality YUYV stream; secondary cameras use
# compressed MJPEG so everything fits on the bus. Tried in order; the first
# format that delivers a frame wins.
ROAD_CAM_FORMATS = ((None, 1280, 720), ('MJPG', 640, 480))          # YUYV first for quality
SECONDARY_CAM_FORMATS = (('MJPG', 640, 480), ('MJPG', 1280, 720), (None, 1280, 720))

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
      # cam_id is either a numeric index (/dev/videoN) or an explicit device
      # path (e.g. /dev/v4l/by-id/... bound by the launcher); don't prepend
      # /dev/video to an already-absolute path.
      cam_id = str(c.cam_id)
      if platform.system() == "Darwin" or cam_id.startswith("/"):
        cam_device = cam_id
      else:
        cam_device = f"/dev/video{cam_id}"
      cam = Camera(c.msg_name, c.stream_type, cam_device, c.formats)
      self.cameras.append(cam)
      self.vipc_server.create_buffers(c.stream_type, 20, cam.W, cam.H)

    self.vipc_server.start_listener()

  def _send_yuv(self, yuv, frame_id, pub_type, yuv_type):
    eof = int(frame_id * 0.05 * 1e9)
    self.vipc_server.send(yuv_type, yuv, frame_id, eof, eof)
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
    rk = Ratekeeper(20, None)
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
