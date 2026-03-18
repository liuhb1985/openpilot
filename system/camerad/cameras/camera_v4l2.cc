#include "system/camerad/cameras/camera_v4l2.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <cassert>
#include <cstring>
#include <errno.h>

#include "common/swaglog.h"
#include "common/util.h"

#define V4L2_BUF_COUNT 4

void CameraState::camera_open(MultiCameraState *multi_cam_state, int camera_num, bool enabled) {
  this->multi_cam_state = multi_cam_state;
  this->camera_num = camera_num;
  this->enabled = enabled;

  if (!enabled) return;

  // Try to find the webcam
  const char* video_dev = nullptr;
  if (camera_num == 0) video_dev = "/dev/video0";
  else if (camera_num == 1) video_dev = "/dev/video1";
  else video_dev = "/dev/video2";

  v4l_fd.reset(open(video_dev, O_RDWR | O_NONBLOCK));
  if (v4l_fd < 0) {
    LOGE("failed to open %s: %s", video_dev, strerror(errno));
    this->enabled = false;
    return;
  }

  struct v4l2_capability cap;
  if (ioctl(v4l_fd, VIDIOC_QUERYCAP, &cap) < 0) {
    LOGE("failed to query capability: %s", strerror(errno));
    this->enabled = false;
    return;
  }

  struct v4l2_format fmt = {0};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = 1280;
  fmt.fmt.pix.height = 720;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; // Common for webcams
  fmt.fmt.pix.field = V4L2_FIELD_ANY;

  if (ioctl(v4l_fd, VIDIOC_S_FMT, &fmt) < 0) {
    LOGE("failed to set format: %s", strerror(errno));
    this->enabled = false;
    return;
  }

  this->width = fmt.fmt.pix.width;
  this->height = fmt.fmt.pix.height;
  this->pix_fmt = fmt.fmt.pix.pixelformat;

  ci.frame_width = width;
  ci.frame_height = height;
  ci.frame_stride = width * 2; // YUYV is 2 bytes per pixel
}

void CameraState::camera_init(VisionIpcServer *v, int camera_id, unsigned int fps, cl_device_id device_id, cl_context ctx, VisionStreamType yuv_type) {
  this->camera_id = camera_id;
  if (!enabled) return;

  buf.init(device_id, ctx, this, v, V4L2_BUF_COUNT, yuv_type);

  struct v4l2_requestbuffers req = {0};
  req.count = V4L2_BUF_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(v4l_fd, VIDIOC_REQBUFS, &req) < 0) {
    LOGE("failed to request buffers: %s", strerror(errno));
    return;
  }
}

void CameraState::camera_close() {
  if (!enabled) return;
  v4l_fd.reset();
}

static void* v4l2_thread(MultiCameraState *s, CameraState *c) {
  util::set_thread_name("v4l2_thread");

  struct v4l2_requestbuffers req = {0};
  req.count = V4L2_BUF_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  void* bufs[V4L2_BUF_COUNT];
  for (int i = 0; i < V4L2_BUF_COUNT; i++) {
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (ioctl(c->v4l_fd, VIDIOC_QUERYBUF, &buf) < 0) continue;

    bufs[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, c->v4l_fd, buf.m.offset);
    ioctl(c->v4l_fd, VIDIOC_QBUF, &buf);
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(c->v4l_fd, VIDIOC_STREAMON, &type);

  uint32_t frame_id = 0;
  while (!do_exit) {
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(c->v4l_fd, VIDIOC_DQBUF, &buf) < 0) {
      if (errno == EAGAIN) {
        usleep(1000);
        continue;
      }
      break;
    }

    // Copy to visionipc buffer
    if (buf.index < V4L2_BUF_COUNT) {
      memcpy(c->buf.camera_bufs[buf.index].addr, bufs[buf.index], buf.bytesused);
      c->buf.camera_bufs_metadata[buf.index] = {
        .frame_id = frame_id++,
        .timestamp_eof = (uint64_t)buf.timestamp.tv_sec * 1000000000ULL + buf.timestamp.tv_usec * 1000ULL,
      };
      c->buf.queue(buf.index);
    }

    ioctl(c->v4l_fd, VIDIOC_QBUF, &buf);
  }

  ioctl(c->v4l_fd, VIDIOC_STREAMOFF, &type);
  for (int i = 0; i < V4L2_BUF_COUNT; i++) {
    munmap(bufs[i], c->ci.frame_stride * c->ci.frame_height);
  }

  return NULL;
}

void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx) {
  s->road_cam.camera_init(v, RoadCam, 20, device_id, ctx, VisionStreamType::VISION_STREAM_ROAD);
  s->driver_cam.camera_init(v, DriverCam, 20, device_id, ctx, VisionStreamType::VISION_STREAM_DRIVER);
}

void cameras_open(MultiCameraState *s) {
  s->road_cam.camera_open(s, 0, true);
  s->driver_cam.camera_open(s, 1, false); // Default off
}

void cameras_run(MultiCameraState *s) {
  std::thread road_cam_thread, driver_cam_thread;
  if (s->road_cam.enabled) road_cam_thread = std::thread(v4l2_thread, s, &s->road_cam);
  if (s->driver_cam.enabled) driver_cam_thread = std::thread(v4l2_thread, s, &s->driver_cam);

  if (road_cam_thread.joinable()) road_cam_thread.join();
  if (driver_cam_thread.joinable()) driver_cam_thread.join();
}

void cameras_close(MultiCameraState *s) {
  s->road_cam.camera_close();
  s->driver_cam.camera_close();
}
