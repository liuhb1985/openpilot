#pragma once

#include <cstdint>
#include <mutex>

#include "system/camerad/cameras/camera_common.h"
#include "common/util.h"

#define FRAME_BUF_COUNT 4

class CameraState {
public:
  MultiCameraState *multi_cam_state;
  CameraInfo ci;
  bool enabled;

  std::mutex exp_lock;

  int camera_id;
  int camera_num;

  unique_fd v4l_fd;
  int width, height;
  unsigned int pix_fmt;

  float cur_ev[3];
  float min_ev, max_ev;

  CameraBuf buf;

  void camera_open(MultiCameraState *multi_cam_state, int camera_num, bool enabled);
  void camera_init(VisionIpcServer *v, int camera_id, unsigned int fps, cl_device_id device_id, cl_context ctx, VisionStreamType yuv_type);
  void camera_close();
};

typedef struct MultiCameraState {
  CameraState road_cam;
  CameraState wide_road_cam;
  CameraState driver_cam;

  PubMaster *pm;
} MultiCameraState;
