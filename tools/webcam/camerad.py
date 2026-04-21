import threading
import os
import sys
from pathlib import Path
from collections import namedtuple

# Fix broken symlinks on Windows/WSL
BASE_DIR = Path(__file__).resolve().parent.parent.parent
for s in ['msgq', 'tinygrad', 'opendbc', 'rednose', 'teleoprtc']:
  p = BASE_DIR / s
  if p.is_file() and p.stat().st_size < 100:
    repo_path = BASE_DIR / f"{s}_repo"
    if repo_path.is_dir() and str(repo_path) not in sys.path:
      sys.path.insert(0, str(repo_path))

from msgq.visionipc import VisionIpcServer, VisionStreamType
from cereal import messaging

from openpilot.tools.webcam.camera import CameraMJPG
from openpilot.common.realtime import Ratekeeper

ROAD_CAM = os.getenv("ROAD_CAM", "0")
WIDE_CAM = os.getenv("WIDE_CAM")
USE_CAMPATH = os.getenv("USE_CAMPATH")

# PC standard openpilot webcam settings
CameraType = namedtuple("CameraType", ["msg_name", "stream_type", "cam_id"])
CAMERAS = [
  CameraType("roadCameraState", VisionStreamType.VISION_STREAM_ROAD, ROAD_CAM),
]
if WIDE_CAM:
  CAMERAS.append(CameraType("wideRoadCameraState", VisionStreamType.VISION_STREAM_WIDE_ROAD, WIDE_CAM))

class Camerad:
  def __init__(self):
    self.pm = messaging.PubMaster([c.msg_name for c in CAMERAS])
    self.vipc_server = VisionIpcServer("camerad")

    self.cameras = []
    for c in CAMERAS:
      cam_id = str(c.cam_id)
      if cam_id.startswith("/") or cam_id.startswith("pci-"):
        cam_device = cam_id if cam_id.startswith("/") else f"/dev/v4l/by-path/{cam_id}"
      elif USE_CAMPATH:
        cam_device = f"/dev/v4l/by-path/{cam_id}"
      else:
        cam_device = f"/dev/video{cam_id}"

      print(f"opening {c.msg_name} at {cam_device} [USE_CAMPATH={USE_CAMPATH}]")
      try:
        cam = CameraMJPG(c.msg_name, c.stream_type, cam_device)
        self.cameras.append(cam)
        self.vipc_server.create_buffers(c.stream_type, 20, cam.W, cam.H)
      except Exception as e:
        print(f"Error opening camera {c.msg_name}: {e}")
        # 继续运行以便其他摄像头（如果存在）能正常工作，或者让 manager 重启进程
        continue

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
