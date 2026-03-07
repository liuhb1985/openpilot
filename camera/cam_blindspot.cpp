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
#include "json11.hpp"
#include <cstring>
#include <sstream>

using namespace std;
using namespace json11;
using namespace chrono;

#define FW_VERSION "1.0.00"

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
const int LOCAL_SEND_PORT = 4120;
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

    cv::Mat resized_image;
    cv::resize(frame, resized_image, cv::Size(new_w, new_h));
    cv::Mat input_blob = cv::Mat::zeros(YOLO_PIX, YOLO_PIX, frame.type());
    resized_image.copyTo(input_blob(cv::Rect(pad_x, pad_y, new_w, new_h)));
    cv::cvtColor(input_blob, input_blob, cv::COLOR_BGR2RGB);
    input_blob.convertTo(input_blob, CV_32F, 1.0 / 255.0);

    // ---------------- ONNX Runtime ----------------
    std::vector<int64_t> input_shape = {1, 3, YOLO_PIX, YOLO_PIX}; // NCHW
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    std::vector<float> blob_data(1*3*YOLO_PIX*YOLO_PIX);
    for(int c = 0; c < 3; c++)
        for(int y = 0; y < YOLO_PIX; y++)
            for(int x = 0; x < YOLO_PIX; x++)
                blob_data[c*YOLO_PIX*YOLO_PIX + y*YOLO_PIX + x] = input_blob.at<cv::Vec3f>(y, x)[c];

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        blob_data.data(),
        blob_data.size(),
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

// ---------------- 摄像头捕获线程 ----------------
void capture_from_camera(const std::string& device, int cam_id) {
    int fd = open(device.c_str(), O_RDWR);
    if (fd == -1) { std::cerr << "Failed to open " << device << std::endl; return; }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 640;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) { std::cerr << "Failed set fmt " << device << std::endl; close(fd); return; }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 4; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) { std::cerr << "Reqbuf failed " << device << std::endl; close(fd); return; }

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; buf.memory = V4L2_MEMORY_MMAP; buf.index = 0;
    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) { std::cerr << "Querybuf failed " << device << std::endl; close(fd); return; }

    void* buffer = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    if (buffer == MAP_FAILED) { std::cerr << "mmap failed " << device << std::endl; close(fd); return; }

    if (ioctl(fd, VIDIOC_STREAMON, &buf.type) == -1) { std::cerr << "streamon failed " << device << std::endl; close(fd); return; }

    // ================== 新增：跳帧控制 ==================
    auto last_time = std::chrono::steady_clock::now();
    const double target_fps = 20.0;
    const double frame_interval_ms = 1000.0 / target_fps;

    while (running) {
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1 || ioctl(fd, VIDIOC_DQBUF, &buf) == -1) continue;

        // 跳帧逻辑
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double, std::milli>(now - last_time).count();
        if (elapsed < frame_interval_ms) continue; // 跳帧
        last_time = now;

        unsigned char* data = (unsigned char*)buffer;
        if (data[0] != 0xFF || data[1] != 0xD8) continue;

        std::vector<uchar> jpeg_data(data, data + buf.bytesused);
        cv::Mat img = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

        {
            std::lock_guard<std::mutex> lock(*queue_mutexes[cam_id]);
            frame_queues[cam_id].push({img, cam_id});
        }
        queue_conds[cam_id]->notify_one();
    }

    ioctl(fd, VIDIOC_STREAMOFF, &buf.type);
    munmap(buffer, buf.length);
    close(fd);
}

// 推理线程函数
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

        cv::Mat img = data.frame;

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
    const int drag_threshold = 5;   // ⭐ 新增拖动阈值
    static cv::Point drag_start;
    static bool dragging = false;

    static std::chrono::steady_clock::time_point last_save_time = std::chrono::steady_clock::now();
    bool changed = false;

    if (event == cv::EVENT_LBUTTONDOWN) {
        // 选择最近顶点拖动
        roi.selected_idx = -1;
        for (int i = 0; i < roi.polygon.size(); i++) {
            if (distance(roi.polygon[i], cv::Point(x, y)) < select_radius) {
                roi.selected_idx = i;
                drag_start = cv::Point(x, y);
                dragging = true;
                return;
            }
        }
        // 没有选中顶点，新增顶点
        // ⭐ 只有按住 Ctrl 才允许新增点
        if (flags & cv::EVENT_FLAG_CTRLKEY) {
            roi.polygon.push_back(cv::Point(x, y));
            changed = true;
        }
        // roi.polygon.push_back(cv::Point(x, y));
        // changed = true;
    }
    else if (event == cv::EVENT_MOUSEMOVE) {
        if (dragging && roi.selected_idx != -1 &&
            (flags & cv::EVENT_FLAG_LBUTTON)) {

            int dx = std::abs(x - drag_start.x);
            int dy = std::abs(y - drag_start.y);

            // ⭐ 超过阈值才更新
            if (dx > drag_threshold || dy > drag_threshold) {
                roi.polygon[roi.selected_idx] = cv::Point(x, y);
                drag_start = cv::Point(x, y);
                changed = true;
            }
        }
    }
    else if (event == cv::EVENT_LBUTTONUP) {
        dragging = false;
        roi.selected_idx = -1;
    }
    else if (event == cv::EVENT_RBUTTONDOWN) {
        // 删除最近顶点
        // ⭐ Ctrl + 右键才删除
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
    const int interval_ms = 100;  // 刷新间隔 100ms
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

                int width = shared_images[valid_index].cols;
                int height = shared_images[valid_index].rows;
                int type = shared_images[valid_index].type();

                int cols;
                if (cam_count == 1) cols = 1;
                else if (cam_count <= 4) cols = 2;
                else cols = 3;

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
                    if ((roi.x + roi.width) <= combined.cols && (roi.y + roi.height) <= combined.rows) {
                        img.copyTo(combined(roi));
                        has_any_frame = true;
                    }
                }

                if (has_any_frame && combined.cols > 0 && combined.rows > 0) {
                    cv::imshow("All Cameras", combined);

                    // 为单窗口模式注册鼠标回调
                    static std::array<int, 2> single_window_roi_ids;
                    single_window_roi_ids[0] = g_current_cam_id;
                    single_window_roi_ids[1] = g_current_roi_idx;
                    cv::setMouseCallback("All Cameras", mouse_callback, single_window_roi_ids.data());
                }
            }
            else {
                for (int i = 0; i < shared_images.size(); ++i) {
                    if (shared_images[i].empty()) continue;
                    std::string window_name = "Camera " + std::to_string(i);
                    cv::imshow(window_name, shared_images[i]);

                    // 为每个摄像头创建ROI区域数据（用于多窗口模式切换编辑）
                    static std::vector<std::array<int, 2>> cam_roi_ids;
                    if(cam_roi_ids.size() <= i) cam_roi_ids.resize(i + 1);

                    // 存储当前摄像头的ID和当前编辑的ROI索引
                    cam_roi_ids[i][0] = i;  // cam_id
                    cam_roi_ids[i][1] = (g_current_cam_id == i) ? g_current_roi_idx : 0;  // roi_idx

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
                std::cout << (lane_safe[0] ? "left lane safe!" : "left lane unsafe!") << std::endl;
            }

            if(lane_safe[1] != right_status){
                lane_safe[1] = right_status;
                std::cout << (lane_safe[1] ? "right lane safe!" : "right lane unsafe!") << std::endl;
            }
        }

        // 控制线程循环频率，例如 50ms 一次 (20Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

        /* ================= 周期性 0.05s 主动发送 (20Hz) ================= */
        if (!comm.last_ip.empty() &&
            chrono::duration_cast<chrono::milliseconds>(
                now - comm.last_send_time).count() >= 50) {

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
int filter_unusable_cameras(std::vector<std::string> &devices,
                            std::vector<int> &camera_sign,
                            std::vector<int> &car_detect) {
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

        for (size_t i = 0; i < devices.size(); i++) {
            bool ok = probe_device(devices[i]);
            std::cout << "[CHECK] Camera " << devices[i]
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
    int removed = 0;
    for (size_t i = 0; i < devices.size();) {
        if (!probe_device(devices[i])) {
            std::cerr << "[WARN] Camera not available after timeout: "
                      << devices[i] << " — removing\n";

            devices.erase(devices.begin() + i);
            if (i < camera_sign.size()) camera_sign.erase(camera_sign.begin() + i);
            if (i < car_detect.size()) car_detect.erase(car_detect.begin() + i);
            removed++;
        } else {
            i++;
        }
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

    initialize_yolo();

    UDPComm comm;
    if (!udp_comm_init(comm)) {
        std::cerr << "UDP init failed" << endl;
        return 1;
    }

    if (!load_camera_config("camera_blindspot.json")) {
        return -1;
    }

    // === 新增：自动过滤无效摄像头 ===
    if (filter_unusable_cameras(devices, camera_sign, car_detect) == 0) {
        return -1;  // 没有可用摄像头则退出
    }

    std::cout << "Loaded " << devices.size() << " cameras" << std::endl;
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

    load_rois("rois.txt");

    // 创建摄像头采集和推理线程
    std::vector<std::thread> threads;
    for (int i = 0; i < cam_max_num; i++) {
        threads.emplace_back(capture_from_camera, devices[i], i);
        threads.emplace_back(inference_thread, i);
    }

    std::thread lane_thread(lane_check_thread);
    std::thread display_thread(display_loop);
    std::thread udp_thread(udp_comm_thread, std::ref(comm));

    // 等待显示线程结束
    display_thread.join();
    std::cout << "display_loop exit" << std::endl;

    // 通知所有线程退出
    running = false;
    comm.running = false;

    // 通知所有等待条件变量的推理线程
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

    return 0;
}

