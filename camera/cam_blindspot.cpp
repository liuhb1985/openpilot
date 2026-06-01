#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <onnxruntime_cxx_api.h>
#include <cmath>
#include <chrono>
#include <queue>
#include <condition_variable>
#include <deque>
#include <algorithm>
#include <memory>
#include <fstream>
#include <iostream>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/stat.h>
#include <dirent.h>
#include <cstdlib>
#include "json11.hpp"
#include <cstring>
#include <sstream>

using namespace std;
using namespace json11;
using namespace chrono;

#define FW_VERSION "1.0.01"

std::vector<std::string> devices;
std::vector<int> camera_sign;
std::vector<int> car_detect;

// 分别跟踪左右盲区的车辆状态
std::vector<int> camera_car_left;
std::vector<int> camera_car_right;

struct FrameData {
    cv::Mat frame;
    int cam_id;
};

bool left_cam_valid = false;
bool right_cam_valid = false;

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
std::vector<std::queue<FrameData>> frame_queues(MAX_CAM);
// mutex 和 condition_variable 用 unique_ptr 包裹
std::vector<std::unique_ptr<std::mutex>> queue_mutexes(MAX_CAM);
std::vector<std::unique_ptr<std::condition_variable>> queue_conds(MAX_CAM);

// 配置端口
// const int LOCAL_SEND_PORT = 4120;
const int LOCAL_RECV_PORT = 4210;
const int REMOTE_PORT = 4211;

struct UDPComm {
    int recv_sock = -1;
    int send_sock = -1;
    sockaddr_in remote_addr{};
    string last_ip;
    int last_port = 0;
    steady_clock::time_point last_recv_time;
    steady_clock::time_point last_send_time;
    bool running = true;
};

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

#define YOLO_PIX 416

// ---------------- 行车记录仪配置 ----------------
struct DashcamConfig {
    bool enabled = false;
    int record_fps = 20;
    std::string save_dir = "/tmp/dashcam_videos";
    int segment_duration = 300;  // 每段视频时长(秒)
    std::string codec = "MJPG";
    double max_disk_usage_gb = 50.0;
} dashcam_config;

// 行车记录仪状态结构
struct DashcamRecorder {
    std::unique_ptr<cv::VideoWriter> writer;
    std::string current_file;
    steady_clock::time_point segment_start;
    int frame_count = 0;
    int target_fps = 20;
    steady_clock::time_point last_write_time;
    bool is_recording = false;
};

std::vector<DashcamRecorder> dashcam_recorders;
std::mutex dashcam_mutex;
std::atomic<bool> dashcam_running(true);

// ---------------- ROI ----------------
struct CameraROI {
    std::vector<cv::Point> polygon;
    int selected_idx = -1; // 拖动顶点索引
    std::vector<CameraROI> sub_rois; // 支持两个子ROI：左盲区和右盲区
};
std::vector<CameraROI> camera_rois;
std::vector<int> camera_car;
std::vector<int> lane_safe(2,-1);

// ---------------- ONNX Runtime ----------------
Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "YOLO");
Ort::Session* session = nullptr;
Ort::SessionOptions session_options;

void initialize_yolo() {
    try {
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_options.SetIntraOpNumThreads(1);
        const char* model_path = "yolov5s.onnx";
        std::ifstream f("yolov5s.onnx");
        if (!f.good()) {
            model_path = "camera/yolov5s.onnx";
        }
        session = new Ort::Session(env, model_path, session_options);
        dcout << "YOLO model loaded successfully!" << std::endl;
    } catch (const Ort::Exception& e) {
        std::cerr << "Failed to load YOLO model: " << e.what() << std::endl;
        running = false;
    }
}

class LaneDebouncerSingleDirection {
public:
    LaneDebouncerSingleDirection(int window_size = 5) : max_len(window_size), status(true) {}

    // 每帧更新
    void update(bool current_safe) {
        if(status && !current_safe) {
            // 当前是安全状态，但检测到不安全 → 立即切换
            status = false;
            queue.clear(); // 清空滑动窗口
        } else if(!status && current_safe) {
            // 当前是不安全状态，检测到安全 → 放入滑动窗口
            if(queue.size() >= max_len) queue.pop_front();
            queue.push_back(true);

            // 窗口满且全部安全 → 切换为安全
            if(queue.size() == max_len && std::all_of(queue.begin(), queue.end(),
                                                      [](bool v){ return v; })) {
                status = true;
                queue.clear();
            }
        }
        // 当前安全状态且检测安全 → 不用操作
        // 当前不安全状态且检测不安全 → 不用操作
    }

    bool get_status() const { return status; }

private:
    int max_len;
    bool status;              // 当前稳定状态
    std::deque<bool> queue;   // 仅用于不安全 → 安全的滑动窗口
};

LaneDebouncerSingleDirection left_checker(5);
LaneDebouncerSingleDirection right_checker(5);

// ================== 行车记录仪函数前置声明 ==================
std::string generate_video_filename(int cam_id);
double get_directory_size_gb(const std::string& path);
void cleanup_old_videos();
bool init_recorder(DashcamRecorder& recorder, int cam_id, const cv::Mat& frame);
void write_video_frame(DashcamRecorder& recorder, const cv::Mat& frame);
void dashcam_management_thread();

// 全局变量用于ROI编辑状态
int g_current_roi_idx = 0; // 0=左盲区, 1=右盲区
int g_current_cam_id = 0;  // 当前摄像头ID

// ---------------- YOLO 检测 ----------------
struct DetectionResult {
    std::vector<cv::Rect> raw_boxes;   // 原始候选框（只经过 score 阈值过滤）
    std::vector<cv::Rect> final_boxes; // NMS 过滤后的框（更严格，用于判断）
};

// detect_cars 返回两个结果
DetectionResult detect_cars(cv::Mat& frame) {
    DetectionResult result;

    if (frame.empty() || frame.channels() != 3) {
        std::cerr << "[WARN] detect_cars: invalid or empty frame, skip" << std::endl;
        return result;
    }

    int orig_w = frame.cols;
    int orig_h = frame.rows;

    // ---------------- Resize + letterbox + normalize ----------------
    float scale = std::min(float(YOLO_PIX) / orig_w, float(YOLO_PIX) / orig_h);
    int new_w = int(orig_w * scale);
    int new_h = int(orig_h * scale);
    int pad_x = (YOLO_PIX - new_w) / 2;
    int pad_y = (YOLO_PIX - new_h) / 2;

    thread_local cv::Mat input_blob = cv::Mat::zeros(YOLO_PIX, YOLO_PIX, CV_8UC3);
    cv::Mat resized_image;
    cv::resize(frame, resized_image, cv::Size(new_w, new_h));

    input_blob.setTo(cv::Scalar(114, 114, 114)); // YOLO constant padding color
    resized_image.copyTo(input_blob(cv::Rect(pad_x, pad_y, new_w, new_h)));

    // ---------------- ONNX Runtime ----------------
    // 使用 cv::dnn::blobFromImage 进行高效转码 (HWC -> CHW, 归一化, BGR->RGB)
    thread_local cv::Mat blob;
    cv::dnn::blobFromImage(input_blob, blob, 1.0/255.0, cv::Size(YOLO_PIX, YOLO_PIX), cv::Scalar(0,0,0), true, false);

    std::vector<int64_t> input_shape = {1, 3, YOLO_PIX, YOLO_PIX}; // NCHW
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        (float*)blob.data,
        blob.total(),
        input_shape.data(),
        input_shape.size()
    );

    static std::vector<Ort::AllocatedStringPtr> input_names_ptrs;
    static std::vector<const char*> input_names;
    if(input_names.empty()) {
        size_t num_input_nodes = session->GetInputCount();
        for(size_t i=0;i<num_input_nodes;i++){
            auto name_ptr = session->GetInputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
            input_names_ptrs.push_back(std::move(name_ptr));
            input_names.push_back(input_names_ptrs.back().get());
        }
    }

    static std::vector<Ort::AllocatedStringPtr> output_names_ptrs;
    static std::vector<const char*> output_names;
    if(output_names.empty()){
        size_t num_output_nodes = session->GetOutputCount();
        for(size_t i=0;i<num_output_nodes;i++){
            auto name_ptr = session->GetOutputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
            output_names_ptrs.push_back(std::move(name_ptr));
            output_names.push_back(output_names_ptrs.back().get());
        }
    }

    auto output_tensors = session->Run(Ort::RunOptions{nullptr},
                                       input_names.data(), &input_tensor, 1,
                                       output_names.data(), output_names.size());

    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    auto shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    int num_boxes = shape[1];

    std::vector<cv::Rect> all_boxes;
    std::vector<float> scores;

    for(int i=0;i<num_boxes;i++){
        float x = output_data[i*85 + 0];
        float y = output_data[i*85 + 1];
        float w = output_data[i*85 + 2];
        float h = output_data[i*85 + 3];
        float conf = output_data[i*85 + 4];

        int best_class = -1;
        float best_prob = 0.0f;
        for(int c=0;c<80;c++){
            float prob = output_data[i*85 + 5 + c];
            if(prob > best_prob){
                best_prob = prob;
                best_class = c;
            }
        }

        float score = conf * best_prob;
        if(score > raw_conf_threshold){  // 用宽松阈值
            if(best_class==0 || best_class==1 || best_class==2 || best_class==3 || best_class==5 || best_class==7){
                int left   = std::max(0, int((x - w/2 - pad_x)/scale));
                int top    = std::max(0, int((y - h/2 - pad_y)/scale));
                int right  = std::min(orig_w, int((x + w/2 - pad_x)/scale));
                int bottom = std::min(orig_h, int((y + h/2 - pad_y)/scale));

                cv::Rect box(left, top, right-left, bottom-top);
                all_boxes.push_back(box);
                scores.push_back(score);
            }
        }
    }

    // 保存宽松的检测结果（仅用于画框）
    result.raw_boxes = all_boxes;

    // ---------------- NMS ----------------
    std::vector<int> indices;
    cv::dnn::NMSBoxes(all_boxes, scores, nms_conf_threshold, nms_threshold, indices);

    for(auto idx : indices)
        result.final_boxes.push_back(all_boxes[idx]);

    return result;
}


// ---------------- ROI 判断 ----------------
bool is_in_roi(const cv::Rect& box, const std::vector<cv::Point>& polygon) {
    if (polygon.empty()) return false;
    int cx = box.x + box.width / 2;
    int cy = box.y + box.height;
    return cv::pointPolygonTest(polygon, cv::Point(cx, cy), false) >= 0;
}

// 重载函数，用于检查点是否在任何子ROI区域内
bool is_in_any_roi(const cv::Rect& box, const std::vector<CameraROI>& sub_rois) {
    for (const auto& sub_roi : sub_rois) {
        if (is_in_roi(box, sub_roi.polygon)) {
            return true;
        }
    }
    return false;
}

#include "msgq/visionipc/visionipc_client.h"

// ---------------- 摄像头捕获线程 (优化版：从 VisionIPC 读取) ----------------
void capture_from_camera(const std::string& device, int cam_id) {
    // 如果设置了环境变量使用 VisionIPC，则不再直接打开 V4L2
    if (getenv("USE_VISION_IPC")) {
        std::cout << "[cam_blindspot] Using VisionIPC for Camera " << cam_id << std::endl;
        VisionIpcClient client("camerad", VisionStreamType::VISION_STREAM_WIDE_ROAD, false);

        while (running) {
            if (!client.connect(false)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            while (running) {
                VisionBuf *buf = client.recv();
                if (buf == nullptr) continue;

                // static int frame_cnt = 0;
                // if (frame_cnt++ % 200 == 0) {
                //     std::cout << "[cam_blindspot] Received VisionIPC frame: " << buf->width << "x" << buf->height << " len=" << buf->len << std::endl;
                // }

                // [重要] 将 YUV420 转换为 BGR，防止内存越界导致崩溃
                cv::Mat img;
                if (buf->len >= buf->width * buf->height * 3 / 2) {
                    cv::Mat yuv(buf->height * 3 / 2, buf->width, CV_8UC1, buf->addr);
                    cv::cvtColor(yuv, img, cv::COLOR_YUV2BGR_NV12);
                } else {
                    continue;
                }

                if (!img.empty()) {
                    std::lock_guard<std::mutex> lock(*queue_mutexes[cam_id]);
                    frame_queues[cam_id].push({img.clone(), cam_id});
                    queue_conds[cam_id]->notify_one();
                }
            }
        }
        return;
    }

    // --- 以下为原有的 V4L2 兼容代码 ---
    cv::VideoCapture cap(device, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "[cam_blindspot] Error: Could not open camera " << device << std::endl;
        return;
    }

    while (running) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(*queue_mutexes[cam_id]);
            frame_queues[cam_id].push({frame.clone(), cam_id});
        }
        queue_conds[cam_id]->notify_one();
    }
}

void inference_thread(int cam_id) {
    while (running) {
        FrameData data;
        {
            std::unique_lock<std::mutex> lock(*queue_mutexes[cam_id]);
            queue_conds[cam_id]->wait(lock, [&] { return !frame_queues[cam_id].empty() || !running; });
            if (!running) break;
            data = frame_queues[cam_id].back();   // 只取最新帧
            while (!frame_queues[cam_id].empty()) frame_queues[cam_id].pop();
        }

        cv::Mat img = cv::Mat::zeros(360, 640, CV_8UC3);
        // 核心修复：保持 16:9 比例，避免图像拉伸变形
        if (!data.frame.empty()) {
            cv::resize(data.frame, img, cv::Size(640, 360), 0, 0, cv::INTER_LINEAR);
        }

        // 空帧防护
        if (img.empty() || img.cols < 10 || img.rows < 10) {
            std::cerr << "[WARN] Camera " << cam_id << ": empty or invalid frame, skip inference" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (car_detect[cam_id] > 0) {
            try {
                // ---------------- YOLO 推理 + ROI ----------------
                auto result = detect_cars(img);
                std::vector<cv::Rect> cars = result.raw_boxes;
                std::vector<cv::Rect> cars_box = result.final_boxes;
                std::vector<cv::Rect> cars_in_roi;
                std::vector<cv::Rect> cars_box_in_roi;

                if (cam_id < camera_rois.size()) {
                    // 初始化左右盲区检测标志
                    int car_in_left = 0;
                    int car_in_right = 0;

                    for (auto &car : cars) {
                        // 检查车辆是否在任何子ROI区域内，并标记对应的状态
                        for (size_t roi_idx = 0; roi_idx < camera_rois[cam_id].sub_rois.size(); roi_idx++) {
                            const auto& sub_roi = camera_rois[cam_id].sub_rois[roi_idx];

                            if (is_in_roi(car, sub_roi.polygon)) {
                                cars_in_roi.push_back(car); // 只要在一个ROI内就加入显示列表
                                if (roi_idx == 0) { // 左盲区
                                    car_in_left = 1;
                                } else if (roi_idx == 1) { // 右盲区
                                    car_in_right = 1;
                                }
                            }
                        }
                    }

                    {
                        std::lock_guard<std::mutex> lock(lane_mutex);
                        camera_car_left[cam_id] = car_in_left;
                        camera_car_right[cam_id] = car_in_right;
                    }
                } else {
                    cars_in_roi = cars;
                }

                if (cam_id < camera_rois.size()) {
                    for (auto &car : cars_box) {
                        if (is_in_any_roi(car, camera_rois[cam_id].sub_rois)) {
                            cars_box_in_roi.push_back(car);
                        }
                    }
                }

                // 绘制所有子ROI区域
                if (cam_id < camera_rois.size() && !camera_rois[cam_id].sub_rois.empty()) {
                    for (size_t roi_idx = 0; roi_idx < camera_rois[cam_id].sub_rois.size(); roi_idx++) {
                        const auto& sub_roi = camera_rois[cam_id].sub_rois[roi_idx];
                        if (!sub_roi.polygon.empty()) {
                            if (roi_idx == 0) {
                                // 左盲区用绿色
                                cv::polylines(img, std::vector<std::vector<cv::Point>>{sub_roi.polygon}, true, cv::Scalar(0,255,0), 2);
                            } else if (roi_idx == 1) {
                                // 右盲区用蓝色
                                cv::polylines(img, std::vector<std::vector<cv::Point>>{sub_roi.polygon}, true, cv::Scalar(255,0,0), 2);
                            } else {
                                // 其他区域用黄色
                                cv::polylines(img, std::vector<std::vector<cv::Point>>{sub_roi.polygon}, true, cv::Scalar(0,255,255), 2);
                            }
                        }
                    }
                }

                for (auto &car : cars_box_in_roi) {
                    // 防止超出边界的矩形崩溃
                    cv::Rect safe_rect = car & cv::Rect(0, 0, img.cols, img.rows);
                    if (safe_rect.width > 0 && safe_rect.height > 0)
                        cv::rectangle(img, safe_rect, cv::Scalar(0,0,255), 2);
                }

            } catch (const cv::Exception &e) {
                std::cerr << "[ERROR] Camera " << cam_id << ": OpenCV exception during inference: " << e.what() << std::endl;
                continue;
            } catch (const std::exception &e) {
                std::cerr << "[ERROR] Camera " << cam_id << ": Exception during inference: " << e.what() << std::endl;
                continue;
            } catch (...) {
                std::cerr << "[ERROR] Camera " << cam_id << ": Unknown error during inference" << std::endl;
                continue;
            }
        }

        // ---------------- 行车记录仪录制 ----------------
        if (dashcam_config.enabled && cam_id < dashcam_recorders.size()) {
            std::lock_guard<std::mutex> lock(dashcam_mutex);
            auto& recorder = dashcam_recorders[cam_id];

            if (!recorder.is_recording) {
                // 初始化录制器
                if (!init_recorder(recorder, cam_id, img)) {
                    std::cerr << "[Dashcam] Failed to init recorder for cam" << cam_id << std::endl;
                }
            } else {
                // 写入视频帧
                write_video_frame(recorder, img);

                // 检查是否需要重新初始化（分段后）
                if (!recorder.is_recording) {
                    if (img.cols > 0 && img.rows > 0) {
                        init_recorder(recorder, cam_id, img);
                    }
                }
            }
        }

        // ---------------- 更新共享图像 ----------------
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (cam_id >= shared_images.size()) shared_images.resize(cam_id + 1);
            //shared_images[cam_id] = img.clone();
            shared_images[cam_id] = img;  // 不 clone
        }
    }
}

// ---------------- 保存/加载 ROI ----------------
std::mutex roi_file_mutex;

// ================== 行车记录仪功能 ==================

// 替换路径中的环境变量（如 $LOGNAME）和波浪号（~）
std::string expand_env_vars(const std::string& path) {
    std::string result = path;
    // 1. 处理波浪号 ~
    if (!result.empty() && result[0] == '~') {
        char* home = getenv("HOME");
        if (home) {
            result.replace(0, 1, home);
        }
    }
    // 2. 处理 $ 变量
    size_t pos = 0;
    while ((pos = result.find("$", pos)) != std::string::npos) {
        size_t end = result.find_first_of("/\\", pos + 1);
        if (end == std::string::npos) {
            end = result.length();
        }
        std::string var_name = result.substr(pos + 1, end - pos - 1);
        char* var_value = getenv(var_name.c_str());
        if (var_value) {
            result.replace(pos, end - pos, var_value);
            pos += strlen(var_value);
        } else {
            pos = end;
        }
    }
    return result;
}

// 创建目录（递归，类似 mkdir -p）
bool create_directory(const std::string& path) {
    std::string expanded_path = expand_env_vars(path);
    if (expanded_path.empty()) return false;

    char tmp[1024];
    snprintf(tmp, sizeof(tmp), "%s", expanded_path.c_str());
    size_t len = strlen(tmp);
    if (tmp[len - 1] == '/') tmp[len - 1] = 0;

    // 逐级创建
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = 0;
            mkdir(tmp, 0755);
            *p = '/';
        }
    }
    return mkdir(tmp, 0755) == 0 || errno == EEXIST;
}

// 生成视频文件名：cam0_20240218_143025.mp4
std::string generate_video_filename(int cam_id) {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);

    std::string expanded_dir = expand_env_vars(dashcam_config.save_dir);
    std::stringstream ss;
    ss << expanded_dir << "/cam" << cam_id << "_"
       << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S")
       << ".avi";
    return ss.str();
}

// 获取目录总大小（GB）
double get_directory_size_gb(const std::string& path) {
    double total_size = 0.0;
    std::string expanded_path = expand_env_vars(path);
    DIR* dir = opendir(expanded_path.c_str());
    if (!dir) return 0.0;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_REG) {
            std::string filepath = path + "/" + entry->d_name;
            struct stat st;
            if (stat(filepath.c_str(), &st) == 0) {
                total_size += st.st_size;
            }
        }
    }
    closedir(dir);
    return total_size / (1024.0 * 1024.0 * 1024.0);
}

// 删除旧视频文件以释放磁盘空间
void cleanup_old_videos() {
    if (dashcam_config.max_disk_usage_gb <= 0) return;

    std::string expanded_dir = expand_env_vars(dashcam_config.save_dir);
    double current_size = get_directory_size_gb(dashcam_config.save_dir);
    if (current_size <= dashcam_config.max_disk_usage_gb) return;

    std::vector<std::pair<std::string, time_t>> files;
    DIR* dir = opendir(expanded_dir.c_str());
    if (!dir) return;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_REG) {
            std::string filepath = expanded_dir + "/" + entry->d_name;
            struct stat st;
            if (stat(filepath.c_str(), &st) == 0) {
                files.emplace_back(filepath, st.st_mtime);
            }
        }
    }
    closedir(dir);

    // 按修改时间排序（旧文件在前）
    std::sort(files.begin(), files.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; });

    // 删除旧文件直到满足空间限制
    for (const auto& file : files) {
        if (current_size <= dashcam_config.max_disk_usage_gb * 0.9) {
            break;
        }
        struct stat st;
        if (stat(file.first.c_str(), &st) == 0) {
            if (unlink(file.first.c_str()) == 0) {
                current_size -= st.st_size / (1024.0 * 1024.0 * 1024.0);
                std::cout << "[Dashcam] Deleted old video: " << file.first << std::endl;
            }
        }
    }
}

// 初始化录制器
bool init_recorder(DashcamRecorder& recorder, int cam_id, const cv::Mat& frame) {
    if (frame.empty()) return false;

    std::string filename = generate_video_filename(cam_id);
    int fourcc = cv::VideoWriter::fourcc(dashcam_config.codec[0],
                                          dashcam_config.codec[1],
                                          dashcam_config.codec[2],
                                          dashcam_config.codec[3]);

    recorder.writer = std::make_unique<cv::VideoWriter>(
        filename, fourcc, dashcam_config.record_fps, frame.size());

    if (!recorder.writer->isOpened()) {
        std::cerr << "[Dashcam] Failed to open video writer for cam" << cam_id << std::endl;
        return false;
    }

    recorder.current_file = filename;
    recorder.segment_start = steady_clock::now();
    recorder.frame_count = 0;
    recorder.last_write_time = steady_clock::now();
    recorder.is_recording = true;

    std::cout << "[Dashcam] Started recording: " << filename << std::endl;
    return true;
}

// 写入视频帧
void write_video_frame(DashcamRecorder& recorder, const cv::Mat& frame) {
    if (!recorder.writer || !recorder.writer->isOpened() || frame.empty()) {
        return;
    }

    // 帧率控制
    auto now = steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(
        now - recorder.last_write_time).count();
    double frame_interval_ms = 1000.0 / recorder.target_fps;

    if (elapsed_ms < frame_interval_ms) {
        return;
    }

    recorder.writer->write(frame);
    recorder.frame_count++;
    recorder.last_write_time = now;

    // 检查是否需要分段（时间限制）
    auto segment_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - recorder.segment_start).count();

    if (segment_elapsed >= dashcam_config.segment_duration) {
        std::cout << "[Dashcam] Segment completed: " << recorder.current_file
                  << " (" << recorder.frame_count << " frames)" << std::endl;
        recorder.writer.release();
        recorder.is_recording = false;
    }
}

// 行车记录仪线程（定期清理和检查）
void dashcam_management_thread() {
    while (dashcam_running) {
        std::this_thread::sleep_for(std::chrono::seconds(60));

        if (dashcam_config.enabled) {
            cleanup_old_videos();
        }
    }
}

void save_rois_threadsafe(const std::string& filename){
    std::lock_guard<std::mutex> lock(roi_file_mutex);
    std::ofstream ofs(filename);
    for(auto& roi: camera_rois){
        for(auto& pt: roi.polygon) ofs << pt.x << " " << pt.y << " ";
        ofs << "\n";
    }
}

void save_rois(const std::string& filename){
    std::ofstream ofs(filename);
    // 保存每个摄像头的所有子ROI区域
    for(const auto& cam_roi : camera_rois){
        for (const auto& sub_roi : cam_roi.sub_rois) {
            if (!sub_roi.polygon.empty()) {
                for(auto& pt: sub_roi.polygon) ofs<<pt.x<<" "<<pt.y<<" ";
            } else {
                // 空 ROI 保存为空行，保持行数对应关系
            }
            ofs<<"\n";
        }
    }
}
void load_rois(const std::string& filename){
    std::ifstream ifs(filename);
    if(!ifs.is_open()) return;

    std::string line;

    // 按行读取，假定是为每个摄像头定义的ROI区域交替出现
    // 第一行是摄像头0的左盲区，第二行是摄像头0的右盲区
    // 第三行是摄像头1的左盲区，第四行是摄像头1的右盲区，以此类推
    std::vector<std::vector<cv::Point>> temp_polygons;
    while(std::getline(ifs,line)){
        // 跳过空行
        if(line.find_first_not_of(" \t\r\n") == std::string::npos) continue;

        std::vector<cv::Point> polygon;
        std::istringstream iss(line);
        int x,y;
        while(iss>>x>>y) polygon.push_back(cv::Point(x,y));
        if (!polygon.empty()) {
            temp_polygons.push_back(polygon);
        }
    }

    // 确保至少有 cam_max_num 个摄像头
    if(camera_rois.size() < cam_max_num) {
        camera_rois.resize(cam_max_num);
    }

    // 每两个ROI组成一个摄像头的左右盲区
    for (size_t cam_idx = 0; cam_idx < temp_polygons.size() / 2; cam_idx++) {
        if(cam_idx >= camera_rois.size()) {
            camera_rois.resize(cam_idx + 1);
        }

        // 确保该摄像头有至少2个子ROI
        if(camera_rois[cam_idx].sub_rois.size() < 2) {
            camera_rois[cam_idx].sub_rois.resize(2);
        }

        // 更新左盲区 (索引0)
        size_t poly_idx = cam_idx * 2;
        if(poly_idx < temp_polygons.size()) {
            camera_rois[cam_idx].sub_rois[0].polygon = temp_polygons[poly_idx];
        }

        // 更新右盲区 (索引1)
        poly_idx = cam_idx * 2 + 1;
        if(poly_idx < temp_polygons.size()) {
            camera_rois[cam_idx].sub_rois[1].polygon = temp_polygons[poly_idx];
        }
    }

    std::cout << "[ROI] Loaded " << temp_polygons.size() / 2 << " cameras from rois.txt" << std::endl;
}

// ---------------- ROI 鼠标回调 ----------------
void mouse_callback(int event, int x, int y, int flags, void* userdata) {
    // userdata包含摄像头ID
    int* data = reinterpret_cast<int*>(userdata);
    int cam_id = data[0];  // 摄像头ID

    // 在多窗口模式下，使用全局变量 g_current_roi_idx 来决定编辑哪个盲区
    // 首先检查当前窗口是否是正在编辑的摄像头
    int roi_idx;
    if (single_window) {
        // 单窗口模式：从userdata获取roi_idx
        roi_idx = data[1];
    } else {
        // 多窗口模式：检查是否是当前编辑的摄像头
        if (cam_id != g_current_cam_id) {
            return; // 不是当前编辑的摄像头，忽略鼠标事件
        }
        roi_idx = g_current_roi_idx; // 使用当前编辑的ROI索引
    }

    // 确保有足够的ROI容器
    if (cam_id < 0) return;
    while (camera_rois.size() <= cam_id) {
        camera_rois.emplace_back();
    }

    // 确保该摄像头有足够的ROI区域（至少2个用于左右盲区）
    while (camera_rois[cam_id].sub_rois.size() <= roi_idx) {
        camera_rois[cam_id].sub_rois.emplace_back();
    }

    auto& roi = camera_rois[cam_id].sub_rois[roi_idx];

    auto distance = [](cv::Point a, cv::Point b) { return std::sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)); };
    const int select_radius = 10;

    static std::chrono::steady_clock::time_point last_save_time = std::chrono::steady_clock::now();
    bool changed = false;

    if (event == cv::EVENT_LBUTTONDOWN) {
        // 选择最近顶点拖动
        roi.selected_idx = -1;
        for (int i = 0; i < roi.polygon.size(); i++) {
            if (distance(roi.polygon[i], cv::Point(x, y)) < select_radius) {
                roi.selected_idx = i;
                return;
            }
        }
        // 没有选中顶点，新增顶点
        if (flags & cv::EVENT_FLAG_CTRLKEY) {
            roi.polygon.push_back(cv::Point(x, y));
            changed = true;
        }
    }
    else if (event == cv::EVENT_MOUSEMOVE) {
        if (roi.selected_idx != -1 && (flags & cv::EVENT_FLAG_LBUTTON)) {
            roi.polygon[roi.selected_idx] = cv::Point(x, y);
            changed = true;
        }
    }
    else if (event == cv::EVENT_LBUTTONUP) {
        roi.selected_idx = -1;
    }
    else if (event == cv::EVENT_RBUTTONDOWN) {
        // 删除最近顶点
        if (flags & cv::EVENT_FLAG_CTRLKEY) {
            int idx = -1;
            float min_dist = select_radius;
            for (int i = 0; i < roi.polygon.size(); i++) {
                float d = distance(roi.polygon[i], cv::Point(x, y));
                if (d < min_dist) { min_dist = d; idx = i; }
            }
            if (idx != -1) {
                roi.polygon.erase(roi.polygon.begin() + idx);
                changed = true;
            }
        }
    }

    // 如果有修改，实时保存
    //if (changed) save_rois_threadsafe("rois.txt");
    // 只每 0.5 秒保存一次
    if (changed) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save_time).count() > 500) {
            save_rois("rois.txt");
            last_save_time = now;
        }
    }
}

// ---------------- 显示线程 ----------------
void display_loop() {
    const int interval_ms = 50;  // 刷新间隔 50ms (20fps)
    int key = 0;
    // 使用全局变量 g_current_roi_idx 和 g_current_cam_id

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

                // 恢复 16:9 显示标准，并根据用户需求适当调大窗口 (960x540)
                int width = 960;
                int height = 540;
                int type = shared_images[valid_index].channels() == 1 ? CV_8UC3 : shared_images[valid_index].type();

                // [修复] 统计实际有效帧数（cam_max_num=2 但可能只有1个摄像头有画面）
                int valid_cam_count = 0;
                for (int i = 0; i < cam_count; ++i) {
                    if (!shared_images[i].empty() && shared_images[i].cols > 10 && shared_images[i].rows > 10)
                        valid_cam_count++;
                }
                if (valid_cam_count == 0) goto wait_next;

                // 根据实际有效摄像头数决定拼接列数：单摄像头不拼接，窗口宽度=摄像头宽度
                int cols_n, rows_n;
                if (valid_cam_count == 1) {
                    cols_n = 1;  // 单摄像头：1列1行，不拼接，不留黑边
                    rows_n = 1;
                } else if (valid_cam_count <= 2) {
                    cols_n = 2;
                    rows_n = 1;
                } else if (valid_cam_count <= 4) {
                    cols_n = 2;
                    rows_n = 2;
                } else {
                    cols_n = 3;
                    rows_n = (valid_cam_count + 2) / 3;
                }

                static cv::Mat combined;
                if (combined.empty() || combined.cols != cols_n * width || combined.rows != rows_n * height) {
                    combined = cv::Mat::zeros(rows_n * height, cols_n * width, type);
                } else {
                    combined.setTo(cv::Scalar(0, 0, 0));
                }

                bool has_any_frame = false;
                int frame_idx = 0;  // 有效帧连续序号

                for (int i = 0; i < cam_count; ++i) {
                    if (shared_images[i].empty() || shared_images[i].cols < 10 || shared_images[i].rows < 10)
                        continue;

                    cv::Mat img = shared_images[i];
                    if (img.channels() == 1) {
                        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
                    }

                    // 恢复全画幅显示，确保 ROI 对齐
                    if (img.cols != width || img.rows != height) {
                        cv::resize(img, img, cv::Size(width, height));
                    }

                    // col 决定 x 偏移（左右），row 决定 y 偏移（上下）
                    int col = frame_idx % cols_n;
                    int row = frame_idx / cols_n;
                    cv::Rect roi(col * width, row * height, width, height);
                    img.copyTo(combined(roi));
                    frame_idx++;
                    has_any_frame = true;
                }

                if (has_any_frame && combined.cols > 0 && combined.rows > 0) {
                    // 核心修复：强制窗口容器收缩到与图像一致，解决“图像距离窗口2侧有白边”的问题
                    static bool window_init = false;
                    static int last_cols = 0;
                    static int last_rows = 0;
                    if (!window_init) {
                        cv::namedWindow("All Cameras", cv::WINDOW_NORMAL);
                        cv::setWindowProperty("All Cameras", cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
                        // 将窗口移动到屏幕左下角 (0, 600)，避免遮挡中央 and 上方视野
                        cv::moveWindow("All Cameras", 0, 600);
                        window_init = true;
                    }
                    if (combined.cols != last_cols || combined.rows != last_rows) {
                        cv::resizeWindow("All Cameras", combined.cols, combined.rows);
                        last_cols = combined.cols;
                        last_rows = combined.rows;
                    }
                    cv::imshow("All Cameras", combined);

                    // 为单窗口模式注册鼠标回调
                    static std::array<int, 2> single_window_roi_ids;
                    single_window_roi_ids[0] = g_current_cam_id;
                    single_window_roi_ids[1] = g_current_roi_idx;
                    cv::setMouseCallback("All Cameras", mouse_callback, single_window_roi_ids.data());
                }
            }
            else {
                static int display_tick = 0;
                if (display_tick++ % 200 == 0) {
                    std::cout << "[cam_blindspot] Display heartbeat, active images: " << shared_images.size() << std::endl;
                }

                for (int i = 0; i < (int)shared_images.size(); ++i) {
                    if (shared_images[i].empty() || shared_images[i].cols < 10 || shared_images[i].rows < 10) continue;

                    cv::Mat img = shared_images[i].clone(); // 克隆一份用于显示处理
                    if (img.channels() == 1) {
                        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
                    }

                    // 统一缩放到 960x540 (16:9)，适当调大显示画面
                    if (img.cols != 960 || img.rows != 540) {
                        cv::resize(img, img, cv::Size(960, 540));
                    }

                    std::string window_name = "Camera " + std::to_string(i);
                    try {
                        cv::imshow(window_name, img);
                    } catch (const cv::Exception& e) {
                        std::cerr << "[ERROR] imshow failed for " << window_name << ": " << e.what() << std::endl;
                    }

                    // 为每个摄像头注册鼠标回调
                    static std::vector<std::array<int, 2>> cam_roi_ids;
                    if(cam_roi_ids.size() <= (size_t)i) cam_roi_ids.resize(i + 1);
                    cam_roi_ids[i][0] = i;
                    cam_roi_ids[i][1] = (g_current_cam_id == i) ? g_current_roi_idx : 0;
                    cv::setMouseCallback(window_name, mouse_callback, cam_roi_ids[i].data());
                }
            }
        }

        static int last_key = -1;
        key = cv::waitKey(1);
        // 防止按键连发
        if (key == last_key)
            key = -1;
        last_key = key;

        if (key == 27) { // ESC
            dcout << "display_loop end" << std::endl;
            running = false;
        }

        // 键盘快捷键切换编辑状态
        if (key == 'l' || key == 'L') {
            g_current_roi_idx = 0; // 切换到左盲区
            std::cout << "[Edit] Camera " << g_current_cam_id << ": Switched to LEFT BLIND SPOT" << std::endl;
        } else if (key == 'r' || key == 'R') {
            g_current_roi_idx = 1; // 切换到右盲区
            std::cout << "[Edit] Camera " << g_current_cam_id << ": Switched to RIGHT BLIND SPOT" << std::endl;
        } else if (key == 'n' || key == 'N') {
            // 切换摄像头
            g_current_cam_id = (g_current_cam_id + 1) % shared_images.size();
            g_current_roi_idx = 0; // 切换摄像头时重置为左盲区
            std::cout << "[Edit] Switched to Camera " << g_current_cam_id << " [LEFT BLIND]" << std::endl;
            std::cout << "[Edit] Tip: Press 'L' to edit LEFT blind, 'R' to edit RIGHT blind" << std::endl;
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

// 线程函数
void lane_check_thread() {
    // 初始化摄像头盲区状态
    // camera_blind_spots.resize(devices.size());

    while (running) {
        // 为每个盲区分别统计检测结果
        std::vector<int> left_blind_detected(devices.size(), 0);   // 每个摄像头在左盲区的检测结果
        std::vector<int> right_blind_detected(devices.size(), 0);  // 每个摄像头在右盲区的检测结果

        // 上锁保护共享数据
        {
            std::lock_guard<std::mutex> lock(lane_mutex);

            for(int cam_id = 0; cam_id < camera_car_left.size(); cam_id++){
                if (cam_id >= camera_rois.size()) {
                    continue;
                }

                // 检查该摄像头的左盲区状态
                if(camera_car_left[cam_id] > 0){
                    left_blind_detected[camera_sign[cam_id]] |= 1 << cam_id;
                }

                // 检查该摄像头的右盲区状态
                if(camera_car_right[cam_id] > 0){
                    right_blind_detected[camera_sign[cam_id]] |= 1 << cam_id;
                }
            }
        }

        // 分别计算所有摄像头的整体左右盲区的安全状态
        bool any_left_unsafe = false;
        bool any_right_unsafe = false;

        for(size_t i = 0; i < left_blind_detected.size(); i++){
            if(left_blind_detected[i] > 0){
                any_left_unsafe = true;
            }
            if(right_blind_detected[i] > 0){
                any_right_unsafe = true;
            }
        }

        // 更新消抖
        left_checker.update(!any_left_unsafe);
        right_checker.update(!any_right_unsafe);

        bool left_status = left_checker.get_status();
        bool right_status = right_checker.get_status();

        // 更新 lane_safe 并打印
        {
            std::lock_guard<std::mutex> lock(lane_mutex);

            if(lane_safe[0] != left_status){
                lane_safe[0] = left_status;
                static auto last_print_left = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_left).count() >= 1) {
                    std::cout << (lane_safe[0] ? "left lane safe!" : "left lane unsafe!") << std::endl;
                    last_print_left = now;
                }
            }

            if(lane_safe[1] != right_status){
                lane_safe[1] = right_status;
                static auto last_print_right = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_right).count() >= 1) {
                    std::cout << (lane_safe[1] ? "right lane safe!" : "right lane unsafe!") << std::endl;
                    last_print_right = now;
                }
            }
        }

        // 控制线程循环频率，例如 100ms 一次
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
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
    // 行车记录仪配置
    // -------------------------------
    if (json["dashcam"].is_object()) {
        auto dashcam = json["dashcam"].object_items();
        if (dashcam.count("enabled") && dashcam["enabled"].is_bool()) {
            dashcam_config.enabled = dashcam["enabled"].bool_value();
            std::cout << "[CFG] dashcam enabled = " << (dashcam_config.enabled ? "true" : "false") << std::endl;
        }
        if (dashcam.count("record_fps") && dashcam["record_fps"].is_number()) {
            dashcam_config.record_fps = dashcam["record_fps"].int_value();
            std::cout << "[CFG] dashcam record_fps = " << dashcam_config.record_fps << std::endl;
        }
        if (dashcam.count("save_dir") && dashcam["save_dir"].is_string()) {
            dashcam_config.save_dir = dashcam["save_dir"].string_value();
            std::cout << "[CFG] dashcam save_dir = " << dashcam_config.save_dir << std::endl;
        }
        if (dashcam.count("segment_duration") && dashcam["segment_duration"].is_number()) {
            dashcam_config.segment_duration = dashcam["segment_duration"].int_value();
            std::cout << "[CFG] dashcam segment_duration = " << dashcam_config.segment_duration << "s" << std::endl;
        }
        if (dashcam.count("codec") && dashcam["codec"].is_string()) {
            dashcam_config.codec = dashcam["codec"].string_value();
            std::cout << "[CFG] dashcam codec = " << dashcam_config.codec << std::endl;
        }
        if (dashcam.count("max_disk_usage_gb") && dashcam["max_disk_usage_gb"].is_number()) {
            dashcam_config.max_disk_usage_gb = dashcam["max_disk_usage_gb"].number_value();
            std::cout << "[CFG] dashcam max_disk_usage_gb = " << dashcam_config.max_disk_usage_gb << std::endl;
        }
    }

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

        if((0 == cam["sign"].int_value()) && (0 != cam["car_detect"].int_value())){
          left_cam_valid = true;
        }
        if((1 == cam["sign"].int_value()) && (0 != cam["car_detect"].int_value())){
          right_cam_valid = true;
        }
    }

    std::cout << "[CFG] Loaded " << devices.size() << " cameras" << std::endl;
    for (size_t i = 0; i < devices.size(); ++i) {
        std::cout << "  Camera[" << i << "]: " << devices[i]
                  << ", sign=" << camera_sign[i]
                  << ", car_detect=" << car_detect[i] << std::endl;
    }

    return true;
}

// 获取本机真实 IP
inline string get_local_ip() {
    string local_ip = "0.0.0.0";
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock >= 0) {
        sockaddr_in serv{};
        serv.sin_family = AF_INET;
        serv.sin_addr.s_addr = inet_addr("8.8.8.8"); // 任意公网IP
        serv.sin_port = htons(53);
        if (connect(sock, (sockaddr*)&serv, sizeof(serv)) == 0) {
            sockaddr_in name{};
            socklen_t namelen = sizeof(name);
            if (getsockname(sock, (sockaddr*)&name, &namelen) == 0) {
                local_ip = inet_ntoa(name.sin_addr);
            }
        }
        close(sock);
    }
    return local_ip;
}

bool udp_comm_init(UDPComm &comm) {
    // 创建接收 socket
    comm.recv_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (comm.recv_sock < 0) { perror("recv socket"); return false; }

    int opt = 1;
    if (setsockopt(comm.recv_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt SO_REUSEADDR");
    }

    if (setsockopt(comm.recv_sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt)) < 0) {
        perror("setsockopt SO_BROADCAST");
    }

    sockaddr_in recv_addr{};
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(LOCAL_RECV_PORT);
    recv_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(comm.recv_sock, (sockaddr*)&recv_addr, sizeof(recv_addr)) < 0) {
        perror("bind"); return false;
    }

    // 创建发送 socket
    comm.send_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (comm.send_sock < 0) { perror("send socket"); return false; }

    opt = 1;
    if (setsockopt(comm.send_sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt)) < 0) {
        perror("setsockopt send SO_BROADCAST");
    }

    comm.last_recv_time = steady_clock::now();
    return true;
}

void udp_comm_thread(UDPComm &comm) {
    std::cout << "[UDP] Thread started" << endl;

    string local_ip = get_local_ip();
    std::cout << "[UDP] local IP: " << local_ip << endl;

    comm.last_recv_time = chrono::steady_clock::now();
    comm.last_send_time = chrono::steady_clock::now();

    while (comm.running) {
        char buffer[4096] = {0};
        sockaddr_in sender_addr{};
        socklen_t sender_len = sizeof(sender_addr);

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(comm.recv_sock, &readfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 50 * 1000; // 50ms tick

        int ret = select(comm.recv_sock + 1, &readfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            perror("[UDP] select error");
            continue;
        }

        /* ================= 接收数据 ================= */
        if (ret > 0 && FD_ISSET(comm.recv_sock, &readfds)) {
            int n = recvfrom(comm.recv_sock, buffer, sizeof(buffer) - 1, 0,
                             (sockaddr *)&sender_addr, &sender_len);
            if (n > 0) {
                buffer[n] = '\0';
                comm.last_recv_time = chrono::steady_clock::now();

                dcout << "[UDP] Received from "
                      << inet_ntoa(sender_addr.sin_addr) << ":"
                      << ntohs(sender_addr.sin_port)
                      << " -> " << buffer << endl;

                string err;
                Json j = Json::parse(buffer, err);
                if (!err.empty()) {
                    cerr << "[UDP] JSON parse error: " << err << endl;
                } else if (j["ip"].is_string() && j["port"].is_number() && j["device"].is_string() && j["device"].string_value() == "op" ) {

                    comm.last_ip = j["ip"].string_value();
                    comm.last_port = j["port"].int_value();

                    comm.remote_addr.sin_family = AF_INET;
                    comm.remote_addr.sin_addr.s_addr =
                        inet_addr(comm.last_ip.c_str());
                    comm.remote_addr.sin_port =
                        htons(comm.last_port);

                    dcout << "[UDP] Update remote -> " << comm.last_ip << ":" << comm.last_port << endl;
                }
            }
        }

        auto now = chrono::steady_clock::now();

        /* ================= 周期性 0.1s 主动发送 ================= */
        if (!comm.last_ip.empty() &&
            chrono::duration_cast<chrono::milliseconds>(
                now - comm.last_send_time).count() >= 100) {

            bool timeout =
                chrono::duration_cast<chrono::seconds>(
                    now - comm.last_recv_time).count() > 5;

            Json::object resp_obj = {
                {"resp", "cam_blind"},
                {"device", "camera"},
                {"timeout", timeout},
                {"ip", local_ip},
                {"port", LOCAL_RECV_PORT},
                {"version", FW_VERSION},
            };

            int detect_side = 0;
            if (left_cam_valid) {
                resp_obj["left_blind"] = !lane_safe[0];
                detect_side |= 1;
            }
            if (right_cam_valid) {
                resp_obj["right_blind"] = !lane_safe[1];
                detect_side |= 2;
            }
            resp_obj["detect_side"] = detect_side;

            Json resp(resp_obj);
            string out = resp.dump();

            sendto(comm.send_sock, out.c_str(), out.size(), 0,
                   (sockaddr *)&comm.remote_addr,
                   sizeof(comm.remote_addr));

            dcout << "[UDP] Periodic send -> " << out << endl;

            comm.last_send_time = now;
        }

        /* ================= 5 秒无接收 → 广播 timeout ================= */
        if (chrono::duration_cast<chrono::seconds>(
                now - comm.last_recv_time).count() > 5) {

            sockaddr_in bcast_addr{};
            bcast_addr.sin_family = AF_INET;
            bcast_addr.sin_port = htons(REMOTE_PORT);
            bcast_addr.sin_addr.s_addr = INADDR_BROADCAST;

            Json timeout_resp = Json::object{
                {"resp", "cam_blind"},
                {"device", "camera"},
                {"timeout", true},
                {"ip", local_ip},
                {"port", LOCAL_RECV_PORT},
                {"version", FW_VERSION},
                {"left_blind", left_cam_valid ? !lane_safe[0] : false},
                {"right_blind", right_cam_valid ? !lane_safe[1] : false},
            };

            string out = timeout_resp.dump();

            sendto(comm.send_sock, out.c_str(), out.size(), 0,
                   (sockaddr *)&bcast_addr,
                   sizeof(bcast_addr));

            dcout << "[UDP] Broadcast timeout -> " << out << endl;

            // 防止每 50ms 重复广播
            comm.last_recv_time = now;
        }
    }

    std::cout << "[UDP] Thread exiting" << endl;
    shutdown(comm.recv_sock, SHUT_RD);
    close(comm.recv_sock);
    close(comm.send_sock);
}

/*
void udp_comm_thread(UDPComm &comm) {
    std::cout << "[UDP] Thread started" << endl;
    string local_ip = get_local_ip();
    std::cout << "[UDP] local IP: " << local_ip << endl;

    while (comm.running) {
        char buffer[4096] = {0};
        sockaddr_in sender_addr{};
        socklen_t sender_len = sizeof(sender_addr);

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(comm.recv_sock, &readfds);
        timeval tv{};
        tv.tv_sec = 1; // 1秒超时检查
        tv.tv_usec = 0;

        int ret = select(comm.recv_sock + 1, &readfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            perror("[UDP] select error");
            continue;
        }

        if (ret > 0 && FD_ISSET(comm.recv_sock, &readfds)) {
            int n = recvfrom(comm.recv_sock, buffer, sizeof(buffer)-1, 0,
                             (sockaddr*)&sender_addr, &sender_len);
            if (n > 0) {
                buffer[n] = '\0';
                comm.last_recv_time = chrono::steady_clock::now();

                dcout << "[UDP] Received packet from "
                     << inet_ntoa(sender_addr.sin_addr) << ":"
                     << ntohs(sender_addr.sin_port)
                     << " -> " << buffer << endl;

                string err;
                Json j = Json::parse(buffer, err);
                if (!err.empty()) {
                    cerr << "[UDP] JSON parse error: " << err << endl;
                    continue;
                }

                if (j["ip"].is_string() && j["port"].is_number()) {
                    comm.last_ip = j["ip"].string_value();
                    comm.last_port = j["port"].int_value();

                    comm.remote_addr.sin_family = AF_INET;
                    comm.remote_addr.sin_addr.s_addr = inet_addr(comm.last_ip.c_str());
                    comm.remote_addr.sin_port = htons(comm.last_port);

                    Json::object resp_obj = {
                        {"resp", "cam_blind"},
                        {"device", "camera"},
                        {"timeout", false},
                        {"ip", local_ip},
                        {"port", LOCAL_RECV_PORT},
                        {"version", FW_VERSION},
                    };

                    int detect_side = 0;
                    if (left_cam_valid) {
                        resp_obj["left_blind"] = !lane_safe[0];
                        detect_side += 1;
                    }
                    if (right_cam_valid) {
                        resp_obj["right_blind"] = !lane_safe[1];
                        detect_side += 2;
                    }

                    resp_obj["detect_side"] = detect_side;

                    Json resp(resp_obj);

                    string out = resp.dump();
                    sendto(comm.send_sock, out.c_str(), out.size(), 0,
                           (sockaddr*)&comm.remote_addr, sizeof(comm.remote_addr));

                    dcout << "[UDP] Sent response to " << comm.last_ip
                         << ":" << comm.last_port
                         << " -> " << out << endl;
                }
            }
        }

        // 超过5秒没有收到数据，广播 timeout
        auto now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::seconds>(now - comm.last_recv_time).count() > 5) {

            sockaddr_in bcast_addr{};
            bcast_addr.sin_family = AF_INET;
            bcast_addr.sin_port = htons(REMOTE_PORT);
            bcast_addr.sin_addr.s_addr = INADDR_BROADCAST;

            Json timeout_resp = Json::object{
                {"resp", "cam_blind"},
                {"device", "camera"},
                {"timeout", true},
                {"ip", local_ip},
                {"port", LOCAL_RECV_PORT},
                {"version", FW_VERSION},
                {"left_blind", lane_safe[0]?false:true},
                {"right_blind", lane_safe[1]?false:true}
            };
            string out = timeout_resp.dump();
            sendto(comm.send_sock, out.c_str(), out.size(), 0,
                   (sockaddr*)&bcast_addr, sizeof(bcast_addr));

            dcout << "[UDP] Broadcast timeout -> " << out << endl;

            comm.last_recv_time = now; // 避免重复广播
        }
    }

    std::cout << "[UDP] Thread exiting" << endl;
    shutdown(comm.recv_sock, SHUT_RD);
    close(comm.recv_sock);
    close(comm.send_sock);
}
*/

// === 新增：检测摄像头可用性，并移除无效项 ===
int filter_unusable_cameras(std::vector<std::string> &f_devices,
                            std::vector<int> &f_camera_sign,
                            std::vector<int> &f_car_detect) {
    auto probe_device = [&](const std::string &dev)->bool {
        // 1. v4l2 ioctl 查询（最快最可靠）
        int fd = open(dev.c_str(), O_RDWR | O_NONBLOCK);
        if (fd >= 0) {
            struct v4l2_capability cap_info;
            if (ioctl(fd, VIDIOC_QUERYCAP, &cap_info) != -1) {
                close(fd);
                return true;
            }
            close(fd);
        }

        // 2. 再尝试用 OpenCV 打开
        cv::VideoCapture cap(dev, cv::CAP_V4L2);
        if (cap.isOpened()) {
            cap.release();
            return true;
        }

        return false;
    };

    const int timeout_sec = 60;
    const int interval_ms = 3000;

    auto start = std::chrono::steady_clock::now();

    // ---------------------------------------------------
    // 尝试等待直到所有摄像头都可用或超时
    // ---------------------------------------------------
    while (true) {
        bool all_ok = true;

        for (size_t i = 0; i < f_devices.size(); i++) {
            bool ok = probe_device(f_devices[i]);
            std::cout << "[CHECK] Camera " << f_devices[i]
                      << " => " << (ok ? "OK" : "FAIL") << std::endl;
            if (!ok) all_ok = false;
        }

        if (all_ok) break;

        // 检查是否超时
        auto now = std::chrono::steady_clock::now();
        int elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        if (elapsed >= timeout_sec) {
            std::cerr << "[ERROR] Not all cameras detected within "
                      << timeout_sec << " seconds.\n";
            break;  // 跳出循环，开始 erase 不可用设备
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        std::cout << "[INFO] Retrying camera detection..." << std::endl;
    }

    // ---------------------------------------------------
    // 最终删除检测失败的摄像头
    // ---------------------------------------------------
    int removed_count = 0;
    for (size_t i = 0; i < f_devices.size();) {
        if (!probe_device(f_devices[i])) {
            std::cerr << "[WARN] Camera not available after timeout: "
                      << f_devices[i] << " — removing\n";

            f_devices.erase(f_devices.begin() + i);
            if (i < f_camera_sign.size()) f_camera_sign.erase(f_camera_sign.begin() + i);
            if (i < f_car_detect.size()) f_car_detect.erase(f_car_detect.begin() + i);
            removed_count++;
        } else {
            i++;
        }
    }

    if (removed_count > 0) {
        std::cout << "[INFO] Removed " << removed_count << " unusable cameras." << std::endl;
    }

    if (devices.empty()) {
        std::cerr << "[ERROR] No usable camera devices.\n";
        return 0;
    }

    std::cout << "[INFO] " << devices.size() << " usable cameras detected." << std::endl;

    return devices.size();
}

// === 新增：捕获 Ctrl+C / 系统关闭信号 ===
#include <csignal>
#include <opencv2/core/ocl.hpp>
void signal_handler(int signo) {
    if (signo == SIGINT || signo == SIGTERM) {
        std::cout << "\n[Signal] Caught termination signal, stopping gracefully..." << std::endl;
        running = false;
    }
}

// ---------------- main ----------------
int main() {
    // 强制关闭 OpenCL，防止 RDNA3 驱动忙等导致 GPU 占用率虚高 (99%)
    cv::ocl::setUseOpenCL(false);

    // 注册信号处理函数
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    initialize_yolo();

    UDPComm comm;
    if (!udp_comm_init(comm)) {
        std::cerr << "UDP init failed" << endl;
        return 1;
    }

    std::string cfg_path = "cam_blindspot.json";
    std::ifstream cfg_f(cfg_path);
    if (!cfg_f.good()) {
        cfg_path = "camera/cam_blindspot.json";
    }
    if (!load_camera_config(cfg_path)) {
        return -1;
    }

    // === 深度优化：VisionIPC 模式下自动注入虚拟设备，绕过物理硬件冲突探测 ===
    if (getenv("USE_VISION_IPC")) {
        std::cout << "[cam_blindspot] VisionIPC mode active. Bypassing physical hardware probe." << std::endl;
        devices.clear();
        devices.push_back("vision_ipc_wide");
        camera_sign.clear();
        camera_sign.push_back(0); // 默认 0 号位
        car_detect.clear();
        car_detect.push_back(1); // 默认开启检测
    } else {
        // 传统模式：过滤无效物理摄像头
        if (filter_unusable_cameras(devices, camera_sign, car_detect) == 0) {
            return -1;  // 没有可用摄像头则退出
        }
    }

    std::cout << "Active cameras: " << devices.size() << std::endl;
    cam_max_num = devices.size();

    // 初始化 mutex / condition_variable
    for (size_t i = 0; i < cam_max_num; i++) {
        queue_mutexes[i] = std::make_unique<std::mutex>();
        queue_conds[i] = std::make_unique<std::condition_variable>();
    }

    shared_images.resize(cam_max_num);
    camera_car_left.resize(cam_max_num, 0);
    camera_car_right.resize(cam_max_num, 0);
    lane_safe[0] = lane_safe[1] = -1;

    camera_rois.resize(cam_max_num);
    for (int i = 0; i < cam_max_num; i++) {
        // 为每个摄像头初始化两个子ROI（左盲区和右盲区）
        CameraROI left_roi;
        left_roi.polygon = {};  // 默认为空，加载文件时会填充

        CameraROI right_roi;
        right_roi.polygon = {};  // 默认为空，加载文件时会填充

        camera_rois[i].sub_rois.push_back(left_roi);
        camera_rois[i].sub_rois.push_back(right_roi);
    }

    std::string rois_path = "rois.txt";
    std::ifstream rois_f(rois_path);
    if (!rois_f.good()) {
        rois_path = "camera/rois.txt";
    }
    load_rois(rois_path);

    // ================== 初始化行车记录仪 ==================
    if (dashcam_config.enabled) {
        // 创建保存目录
        if (!create_directory(dashcam_config.save_dir)) {
            std::cerr << "[Dashcam] Failed to create directory: " << dashcam_config.save_dir << std::endl;
            dashcam_config.enabled = false;
        } else {
            std::cout << "[Dashcam] Initializing for " << cam_max_num << " cameras" << std::endl;
            std::cout << "[Dashcam] Save directory: " << dashcam_config.save_dir << std::endl;
            std::cout << "[Dashcam] Record FPS: " << dashcam_config.record_fps << std::endl;
            std::cout << "[Dashcam] Segment duration: " << dashcam_config.segment_duration << "s" << std::endl;
            std::cout << "[Dashcam] Max disk usage: " << dashcam_config.max_disk_usage_gb << " GB" << std::endl;

            // 初始化每个摄像头的录制器
            dashcam_recorders.resize(cam_max_num);
            for (int i = 0; i < cam_max_num; i++) {
                dashcam_recorders[i].target_fps = dashcam_config.record_fps;
                dashcam_recorders[i].is_recording = false;
            }

            // 清理旧视频
            cleanup_old_videos();
        }
    }

    // 创建摄像头采集和推理线程
    std::vector<std::thread> threads;
    for (int i = 0; i < cam_max_num; i++) {
        threads.emplace_back(capture_from_camera, devices[i], i);
        threads.emplace_back(inference_thread, i);
    }

    std::thread lane_thread(lane_check_thread);
    std::thread display_thread(display_loop);
    std::thread udp_thread(udp_comm_thread, std::ref(comm));
    std::thread dashcam_thread;
    if (dashcam_config.enabled) {
        dashcam_thread = std::thread(dashcam_management_thread);
    }

    // 等待显示线程结束
    display_thread.join();
    std::cout << "display_loop exit" << std::endl;

    // 通知所有线程退出
    running = false;
    comm.running = false;
    dashcam_running = false;

    for (int i = 0; i < cam_max_num; i++)
        queue_conds[i]->notify_all();

    // 等待摄像头采集 + 推理线程退出
    for (auto &t : threads)
        if (t.joinable())
            t.join();
    std::cout << "camera threads exit" << std::endl;

    // 等待车道检测线程退出
    if (lane_thread.joinable())
        lane_thread.join();
    std::cout << "lane_thread exit" << std::endl;

    // 等待 UDP 线程退出
    if (udp_thread.joinable())
        udp_thread.join();
    std::cout << "udp thread exit" << std::endl;

    // 等待行车记录仪线程退出
    if (dashcam_thread.joinable())
        dashcam_thread.join();
    std::cout << "dashcam thread exit" << std::endl;

    // 保存行车记录仪文件
    if (dashcam_config.enabled) {
        std::lock_guard<std::mutex> lock(dashcam_mutex);
        for (int i = 0; i < dashcam_recorders.size(); i++) {
            if (dashcam_recorders[i].writer && dashcam_recorders[i].writer->isOpened()) {
                std::cout << "[Dashcam] Closing recorder for cam" << i
                          << " - " << dashcam_recorders[i].current_file << std::endl;
                dashcam_recorders[i].writer.release();
            }
        }
        std::cout << "[Dashcam] All recordings saved" << std::endl;
    }

    return 0;
}

