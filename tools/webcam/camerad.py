#!/usr/bin/env python3
import threading
import os
import queue
import numpy as np
from collections import namedtuple

from msgq.visionipc import VisionIpcServer, VisionStreamType
from cereal import messaging

from openpilot.tools.webcam.camera import CameraMJPG
from openpilot.common.realtime import Ratekeeper

ROAD_CAM = os.getenv("ROAD_CAM") or "2"
WIDE_CAM = os.getenv("WIDE_CAM") or "0"
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
      if (cam_id.startswith("/") or cam_id.startswith("pci-")) or USE_CAMPATH:
        cam_device = cam_id if cam_id.startswith("/") else f"/dev/v4l/by-path/{cam_id}"
        if not os.path.exists(cam_device) or os.path.isdir(cam_device):
          print(f"[WARN] Camera path {cam_device} not found or is a directory, falling back to default /dev/video{cam_id}")
          cam_device = f"/dev/video{cam_id}"
      else:
        cam_device = f"/dev/video{cam_id}"

      print(f"opening {c.msg_name} at {cam_device}")
      try:
        cam = CameraMJPG(c.msg_name, c.stream_type, cam_device)
        self.cameras.append(cam)
        self.vipc_server.create_buffers(c.stream_type, 20, cam.W, cam.H)
      except Exception as e:
        print(f"Failed to open camera {cam_device}: {e}")

    self.vipc_server.start_listener()

  def _send_yuv(self, yuv, frame_id, pub_type, yuv_type):
    eof = int(frame_id * 0.05 * 1e9)
    # VisionIpcServer.send 要求一维连续缓冲，bgr_to_nv12 返回 (h*3//2, w) 二维数组，需展平
    if yuv.ndim > 1:
      yuv = np.ascontiguousarray(yuv).reshape(-1)
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
    if len(self.cameras) < 2:
      threads = []
      for cam in self.cameras:
        cam_thread = threading.Thread(target=self.camera_runner, args=(cam,))
        cam_thread.start()
        threads.append(cam_thread)
      for t in threads:
        t.join()
      return

    # 双摄必须使用同一个 frame_id 和 timestamp_sof 发布两路流。
    # modeld 会等待两路 VisionIPC 帧时间戳对齐，若各摄像头独立计数会永远等不到，
    # 导致 modelV2 停止发布，系统无法进入控制状态。
    frame_id = 0
    rk = Ratekeeper(20, None)
    last_frames = {}

    while True:
      for cam in self.cameras:
        # 非阻塞取最新帧。某一路暂时无新帧时，只能重复该路自己的最后一帧，
        # 绝不能用另一路摄像头画面占位，否则两路流会交替显示不同摄像头导致画面闪烁。
        try:
          yuv = cam.output_queue.get_nowait()
          cam.output_queue.task_done()
          last_frames[cam] = yuv
        except queue.Empty:
          yuv = last_frames.get(cam)

      if not all(last_frames.get(cam) is not None for cam in self.cameras):
        rk.keep_time()
        continue

      for cam in self.cameras:
        yuv = last_frames[cam]
        if yuv.shape[:2] != (cam.H * 3 // 2, cam.W):
          continue
        self._send_yuv(yuv, frame_id, cam.cam_type_state, cam.stream_type)
      frame_id += 1
      rk.keep_time()


def main():
  camerad = Camerad()
  camerad.run()


if __name__ == "__main__":
  main()
