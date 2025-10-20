#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <csignal>
#include <fstream>
#include <string>
#include "json11.hpp"

#include "msgq/visionipc/visionipc_server.h"
#include "msgq/visionipc/visionbuf.h"
#include "third_party/libyuv/include/libyuv.h"
#include "common/timing.h"

uint64_t get_timestamp_us() {
    return nanos_since_boot() / 1000;  // 纳秒转微秒
}

using namespace std;
using namespace json11;
using namespace chrono;

std::vector<std::string> devices;
std::vector<int> camera_sign;
std::vector<int> car_detect;

bool debug_mode = false;
bool show_video = true;
bool single_window = true;
float raw_conf_threshold = 0.1f;   // 宽松阈值 → 保证画框尽量多
float nms_conf_threshold = 0.1f;   // 严格阈值 → 用于NMS
float nms_threshold      = 0.5f;

std::mutex frame_mutex;
std::mutex lane_mutex;
std::vector<cv::Mat> shared_images;
std::atomic<bool> running(true);

// 每个摄像头一个队列
int cam_max_num = 2;
constexpr int MAX_CAM = 8;

class DebugStream {
public:
    DebugStream(bool enabled = false) : enabled_(enabled) {}
    void set_enabled(bool e) { enabled_ = e; }

    template<typename T>
    DebugStream& operator<<(const T& value) {
        if (enabled_) std::cout << value;
        return *this;
    }

    DebugStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        if (enabled_) std::cout << manip;
        return *this;
    }

private:
    bool enabled_;
};

DebugStream dcout;

// ---------------- 摄像头捕获线程 ----------------
// 修改函数签名，接收共享的 VisionIPC 服务器
void capture_from_camera(const std::string& device, int cam_id, VisionIpcServer* vipc_server, int width, int height) {
    int fd = open(device.c_str(), O_RDWR);
    if (fd == -1) {
        std::cerr << "Failed to open " << device << std::endl;
        return;
    }

    unsigned int frame_counter = 0;

    // 根据 cam_id 选择流类型
    VisionStreamType stream_type;
    if (cam_id == 0) {
        stream_type = VISION_STREAM_ROAD;
    } else if (cam_id == 1) {
        stream_type = VISION_STREAM_WIDE_ROAD;
    } else {
        stream_type = VISION_STREAM_DRIVER;
    }

    // V4L2 格式设置 - 使用传入的分辨率
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        std::cerr << "Failed set fmt " << device << std::endl;
        close(fd);
        return;
    }

    // 设置帧率
    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_PARM, &parm) == -1) {
        std::cerr << "VIDIOC_G_PARM failed\n";
    }
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = 20;
    if (ioctl(fd, VIDIOC_S_PARM, &parm) == -1) {
        std::cerr << "VIDIOC_S_PARM failed\n";
    }

    // 请求多个缓冲区
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        std::cerr << "Reqbuf failed " << device << std::endl;
        close(fd);
        return;
    }

    // 为每个缓冲区执行 mmap
    std::vector<void*> buffers(4);
    std::vector<size_t> buffer_lengths(4);

    for (int i = 0; i < 4; i++) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            std::cerr << "Querybuf failed for buffer " << i << std::endl;
            continue;
        }

        buffers[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        buffer_lengths[i] = buf.length;

        if (buffers[i] == MAP_FAILED) {
            std::cerr << "mmap failed for buffer " << i << std::endl;
            continue;
        }

        // 将缓冲区加入队列
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            std::cerr << "Initial QBUF failed for buffer " << i << std::endl;
        }
    }

    // 启动流
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        std::cerr << "streamon failed " << device << std::endl;
        close(fd);
        return;
    }

    // 分配临时 I420 缓冲区 - 使用正确的尺寸
    uint8_t* y_plane = new uint8_t[width * height];
    uint8_t* u_plane = new uint8_t[width * height / 4];
    uint8_t* v_plane = new uint8_t[width * height / 4];

    while (running) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // 出队
        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            if (errno == EAGAIN) continue;
            std::cerr << "DQBUF failed" << std::endl;
            continue;
        }

        unsigned char* mjpeg_data = (unsigned char*)buffers[buf.index];
        if (mjpeg_data[0] != 0xFF || mjpeg_data[1] != 0xD8) {
            // 重新入队
            ioctl(fd, VIDIOC_QBUF, &buf);
            continue;
        }

        size_t mjpeg_size = buf.bytesused;

        // 更新共享图像用于显示
        if(show_video){
            std::vector<uchar> jpeg_data(mjpeg_data, mjpeg_data + mjpeg_size);
            cv::Mat img = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (cam_id >= shared_images.size())
                    shared_images.resize(cam_id + 1);
                shared_images[cam_id] = img;
            }
        }

        // 获取 VisionIPC 缓冲区 - 使用对应的流类型
        VisionBuf* vipc_buf = vipc_server->get_buffer(stream_type);

        // MJPEG 解码到 I420 - 使用正确的尺寸
        int ret = libyuv::MJPGToI420(
            mjpeg_data, mjpeg_size,
            y_plane, width,
            u_plane, width / 2,
            v_plane, width / 2,
            width, height,
            width, height
        );

        if (ret != 0) {
            std::cerr << "MJPEG decode failed" << std::endl;
            // 重新入队
            ioctl(fd, VIDIOC_QBUF, &buf);
            continue;
        }

        // I420 转 NV12 到 VisionIPC 缓冲区 - 使用正确的地址访问方式
        libyuv::I420ToNV12(
            y_plane, width,
            u_plane, width / 2,
            v_plane, width / 2,
            (uint8_t*)vipc_buf->addr, vipc_buf->stride,
            (uint8_t*)vipc_buf->addr + vipc_buf->uv_offset, vipc_buf->stride,
            width, height
        );

        // 发送帧 - 使用纳秒时间戳
        VisionIpcBufExtra extra = {
            .frame_id = frame_counter++,
            .timestamp_sof = nanos_since_boot(),
            .timestamp_eof = nanos_since_boot(),
        };

        vipc_buf->set_frame_id(extra.frame_id);
        vipc_server->send(vipc_buf, &extra);

        dcout << "send vipc frame " << frame_counter << " on stream " << stream_type << std::endl;

        // 重新入队
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            std::cerr << "QBUF failed" << std::endl;
        }
    }

    // 清理
    delete[] y_plane;
    delete[] u_plane;
    delete[] v_plane;

    ioctl(fd, VIDIOC_STREAMOFF, &type);

    // 释放所有 mmap 的缓冲区
    for (int i = 0; i < 4; i++) {
        if (buffers[i] != MAP_FAILED) {
            munmap(buffers[i], buffer_lengths[i]);
        }
    }

    close(fd);
}

// ---------------- 显示线程 ----------------
void display_loop() {
    const int interval_ms = 100;  // 刷新间隔 100ms
    int key = 0;
    while (running) {
        auto start_time = std::chrono::steady_clock::now();

        if(show_video)
        {
            std::lock_guard<std::mutex> lock(frame_mutex);

            if (single_window) {
                int cam_count = shared_images.size();
                if (cam_count == 0) goto wait_next;

                // 找到第一个有效帧
                int valid_index = -1;
                for (int i = 0; i < cam_count; ++i) {
                    if (!shared_images[i].empty() && shared_images[i].cols > 0 && shared_images[i].rows > 0) {
                        valid_index = i;
                        break;
                    }
                }

                // 没有有效帧，跳过
                if (valid_index == -1) goto wait_next;

                int width = shared_images[valid_index].cols;
                int height = shared_images[valid_index].rows;
                int type = shared_images[valid_index].type();

                int cols = (cam_count > 4) ? 3 : 2;
                if(cam_count == 1) cols=1;
                int rows = (cam_count + cols - 1) / cols;

                cv::Mat combined = cv::Mat::zeros(rows * height, cols * width, type);

                bool has_any_frame = false;

                for (int i = 0; i < cam_count; ++i) {
                    if (shared_images[i].empty() || shared_images[i].cols < 10 || shared_images[i].rows < 10)
                        continue;

                    cv::Mat img = shared_images[i];
                    if (img.channels() == 1) {
                        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
                    }
                    if (img.cols != width || img.rows != height) {
                        cv::resize(img, img, cv::Size(width, height));
                    }

                    int r = i / cols;
                    int c = i % cols;
                    cv::Rect roi(c * width, r * height, width, height);
                    img.copyTo(combined(roi));

                    has_any_frame = true;
                }

                if (has_any_frame && combined.cols > 0 && combined.rows > 0) {
                    cv::imshow("All Cameras", combined);
                }
            }
            else {
                for (int i = 0; i < shared_images.size(); ++i) {
                    if (shared_images[i].empty()) continue;
                    std::string window_name = "Camera " + std::to_string(i);
                    cv::imshow(window_name, shared_images[i]);
                }
            }
        }

        key = cv::waitKey(1);
        if (key == 27) { // ESC
            dcout << "display_loop end" << std::endl;
            running = false;
        }

    wait_next:
        // 计算剩余时间等待
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        auto sleep_time = std::chrono::milliseconds(interval_ms) - elapsed;
        if (sleep_time > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(sleep_time);
    }

    cv::destroyAllWindows();
}

bool load_camera_config(const std::string &filename) {
    dcout.set_enabled(debug_mode);

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << filename << std::endl;
        return false;
    }

    std::string content((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

    std::string err;
    auto json = json11::Json::parse(content, err);
    if (!err.empty()) {
        std::cerr << "JSON parse error: " << err << std::endl;
        return false;
    }

    // -------------------------------
    // 新增: 通用配置字段
    // -------------------------------
    if (json["debug"].is_bool()) {
        debug_mode = json["debug"].bool_value();
        std::cout << "[CFG] debug_mode = " << (debug_mode ? "true" : "false") << std::endl;
    }

    dcout.set_enabled(debug_mode);

    if (json["show_video"].is_bool()) {
        show_video = json["show_video"].bool_value();
        std::cout << "[CFG] show_video = " << (show_video ? "true" : "false") << std::endl;
    }

    if (json["single_window"].is_bool()) {
        single_window = json["single_window"].bool_value();
        std::cout << "[CFG] single_window = " << (single_window ? "true" : "false") << std::endl;
    }

    auto limit01 = [](float v) {
        return std::max(0.0f, std::min(1.0f, v));
    };

    if (json["raw_conf_threshold"].is_number())
        raw_conf_threshold = limit01(json["raw_conf_threshold"].number_value());
    if (json["nms_conf_threshold"].is_number())
        nms_conf_threshold = limit01(json["nms_conf_threshold"].number_value());
    if (json["nms_threshold"].is_number())
        nms_threshold = limit01(json["nms_threshold"].number_value());

    std::cout << "[CFG] raw_conf_threshold = " << raw_conf_threshold
              << ", nms_conf_threshold = " << nms_conf_threshold
              << ", nms_threshold = " << nms_threshold << std::endl;

    // -------------------------------
    // 摄像头配置
    // -------------------------------
    if (!json["cameras"].is_array()) {
        std::cerr << "Invalid config: 'cameras' must be an array" << std::endl;
        return false;
    }

    devices.clear();
    camera_sign.clear();

    for (const auto &cam : json["cameras"].array_items()) {
        devices.push_back(cam["device"].string_value());
        camera_sign.push_back(cam["sign"].int_value());
        car_detect.push_back(cam["car_detect"].int_value());
    }

    std::cout << "[CFG] Loaded " << devices.size() << " cameras" << std::endl;
    for (size_t i = 0; i < devices.size(); ++i) {
        std::cout << "  Camera[" << i << "]: " << devices[i]
                  << ", sign=" << camera_sign[i]
                  << ", car_detect=" << car_detect[i] << std::endl;
    }

    return true;
}

// === 新增：检测摄像头可用性，并移除无效项 ===
int filter_unusable_cameras(std::vector<std::string> &devices, std::vector<int> &camera_sign, std::vector<int> &car_detect) {
    auto probe_device = [&](const std::string &dev)->bool {
        // 尝试用 OpenCV 打开
        cv::VideoCapture cap(dev);
        if (cap.isOpened()) {
            cap.release();
            return true;
        }

        // 尝试底层 /dev/video 设备
        int fd = open(dev.c_str(), O_RDWR | O_NONBLOCK);
        if (fd >= 0) {
            struct v4l2_capability cap_info;
            if (ioctl(fd, VIDIOC_QUERYCAP, &cap_info) != -1) {
                close(fd);
                return true;
            }
            close(fd);
        }
        return false;
    };

    int removed = 0;
    for (size_t i = 0; i < devices.size(); ) {
        if (!probe_device(devices[i])) {
            std::cerr << "[WARN] Cannot open camera: " << devices[i] << " — removing from list\n";
            devices.erase(devices.begin() + i);
            if (i < camera_sign.size()) camera_sign.erase(camera_sign.begin() + i);
            if (i < car_detect.size()) car_detect.erase(car_detect.begin() + i);
            removed++;
        } else {
            ++i;
        }
    }

    if (devices.empty()) {
        std::cerr << "[ERROR] No usable camera devices found.\n";
        return 0;
    }

    std::cout << "[INFO] " << devices.size() << " usable cameras detected." << std::endl;

    return devices.size();
}

// === 新增：捕获 Ctrl+C / 系统关闭信号 ===
#include <csignal>
void signal_handler(int signo) {
    if (signo == SIGINT || signo == SIGTERM) {
        std::cout << "\n[Signal] Caught termination signal, stopping gracefully..." << std::endl;
        running = false;
    }
}

// ---------------- main ----------------
int main() {
    // 注册信号处理函数
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (!load_camera_config("usb_camera.json")) {
        return -1;
    }

    // 自动过滤无效摄像头
    if (filter_unusable_cameras(devices, camera_sign, car_detect) == 0) {
        return -1;
    }

    std::cout << "Loaded " << devices.size() << " cameras" << std::endl;
    cam_max_num = devices.size();

    // 设置分辨率
    int width = 1920;
    int height = 1080;

    // 在主线程中创建共享的 VisionIPC 服务器
    VisionIpcServer* vipc_server = new VisionIpcServer("camerad");

    // 为每个摄像头创建对应的缓冲区
    for (int i = 0; i < cam_max_num; i++) {
        VisionStreamType stream_type;
        if (i == 0) {
            stream_type = VISION_STREAM_ROAD;
        } else if (i == 1) {
            stream_type = VISION_STREAM_WIDE_ROAD;
        } else {
            stream_type = VISION_STREAM_DRIVER;
        }
        vipc_server->create_buffers(stream_type, 3, width, height);
    }

    // 启动监听线程
    vipc_server->start_listener();

    // 创建摄像头采集线程 - 传递共享的 VisionIPC 服务器和分辨率
    std::vector<std::thread> threads;
    for (int i = 0; i < cam_max_num; i++) {
        threads.emplace_back(capture_from_camera, devices[i], i, vipc_server, width, height);
    }

    std::thread display_thread;
    if(show_video){
        display_thread = std::thread(display_loop);
    }

    // 等待线程
    if (display_thread.joinable()) {
        display_thread.join();
        std::cout << "display_loop exit" << std::endl;
    } else {
        // 如果没有显示线程，等待退出信号
        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // 等待摄像头采集线程退出
    for (auto &t : threads) {
        if (t.joinable())
            t.join();
    }
    std::cout << "camera threads exit" << std::endl;

    // 清理
    delete vipc_server;

    return 0;
}

