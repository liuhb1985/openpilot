#ifndef CAMERA_HHH_
#define CAMERA_HHH_

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <turbojpeg.h>
#include <dirent.h>
#include <algorithm>
#include <queue>
#include <mutex>
#include <map>
#include <condition_variable>

#include <assert.h>
#include "common/timing.h"
#include "common/util.h"
#include "cereal/services.h"
#include "cereal/messaging/messaging.h"
#include "msgq/visionipc/visionipc_server.h"
#include "common/clutil.h"
#include "common/ratekeeper.h"


class FrameQueue {
public:
    void push(const std::vector<uint8_t>& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(frame);
        if (queue_.size() > 100) {
            queue_.pop();
        }
    }

    bool pop(std::vector<uint8_t>& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        frame = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    std::queue<std::vector<uint8_t>> queue_;
    mutable std::mutex mutex_;
};

struct FrameInfo {
    uint64_t timestamp_sof;
    bool ready;
};

class camera {
public:
  camera(const char* device, const char* type, int width, int height, int fps, const std::string& output_prefix);

  ~camera();

protected:
  // Open camera
  int open_camera(const char* device);

  bool set_camera_format(int fd, int width, int height, uint32_t pixel_format);

  bool set_frame_rate(int fd, int fps);

  bool set_exposure(int fd, int value);

  bool init_buffer(int fd);

  bool init_mmap(int fd, std::vector<void*>& buffers, std::vector<size_t>& buffer_sizes);

  bool start_capture(int fd);

  void reading_loop(int fd);

  // Save videos.
  void saving_loop();

  void close_camera();

private:
  double calculate_y_average (std::vector<uint8_t> &yuv_data, int width, int height);

  void adjust_exposure(int fd, int min, int max);

public:
  std::string get_vision_type () { return m_type; }
  VisionStreamType get_stream_type() { return m_type_vision; }
  int width() {return m_width;}
  int height() {return m_height;}

  bool init_cam();

  bool run();

  bool read_frame(std::vector<void*>& buffers, std::vector<size_t>& buffer_sizes, void*& out_data, size_t& out_size);

  // Send yuv buffer.
  void send_yuv(uint32_t frame_id, VisionIpcServer &vipc_server);

private:
  const char* m_device;            // device . /dev/video0
  const std::string m_type;        // "roadCameraState"
  VisionStreamType m_type_vision;  // VISION_STREAM_ROAD
  int m_width;
  int m_height;
  int m_fps;
  int m_fd;
  std::string m_output_prefix;    // Path for recording videos.

  tjhandle m_jpeg_handle;
  std::vector<void*> m_buffers;
  std::vector<size_t> m_buffer_sizes;
  std::vector<uint8_t> m_yuv_data;
  uint64_t m_frame_sof;
  uint64_t m_frame_eof;
  PubMaster *m_pm;

  static std::map<VisionStreamType, FrameInfo> m_shared_frame_info;
  static std::mutex m_shared_frame_mutex;
  static std::condition_variable m_frame_ready_cv; // sync read

  FrameQueue m_frame_queue;       // queue for recording.
  std::mutex m_mutex;
  ExitHandler m_do_exit;

  int m_current_exposure;
};

#endif
