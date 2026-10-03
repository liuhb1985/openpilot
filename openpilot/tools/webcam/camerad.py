#!/usr/bin/env python3
import threading
import os
import platform
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


# Cameras usually share one USB 2.0 bus (~53 MB/s). The road camera feeds the
# driving model, so it asks for full-quality YUYV first; secondary cameras start
# with a compressed, smaller stream so both fit on the bus. Tried in order; the
# first format that actually delivers a frame wins.
ROAD_CAM_FORMATS = ((None, 1280, 720), ('MJPG', 640, 480))
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
