#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <atomic>
#include <chrono>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/prctl.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include "cereal/messaging/messaging.h"
#include "common/params.h"
#include "common/util.h"
#include "common/swaglog.h"
#include "msgq/visionipc/visionipc_client.h"

using namespace std;

std::atomic<bool> running(true);
std::mutex frame_lock;
std::vector<uchar> latest_jpeg_rgb;
std::vector<uchar> latest_jpeg_gray;
std::string status_text = "waiting app connect...";
std::mutex status_lock;

const int PORT = 8888;
const int TARGET_WIDTH = 416;
const int TARGET_HEIGHT = 416;
const int JPEG_QUALITY = 50;

void camera_thread() {
  VisionIpcClient vipc_client("camerad", VISION_STREAM_ROAD, false);

  while (running) {
    if (!vipc_client.connect(false)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }
    break;
  }

  LOGW("laned connected to visionipc");

  while (running) {
    VisionIpcBuf *buf = vipc_client.recv();
    if (buf == nullptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    auto start_time = std::chrono::steady_clock::now();

    // Create YUV Mat from buffer
    // Assuming NV12 format as per openpilot standards
    cv::Mat yuv(buf->height + buf->height / 2, buf->stride, CV_8UC1, buf->addr);
    
    cv::Mat bgr;
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);

    // Resize and crop to 416x416
    cv::Mat cropped;
    float scale = std::max((float)TARGET_WIDTH / bgr.cols, (float)TARGET_HEIGHT / bgr.rows);
    cv::resize(bgr, cropped, cv::Size(), scale, scale, cv::INTER_AREA);
    
    int start_x = (cropped.cols - TARGET_WIDTH) / 2;
    int start_y = (cropped.rows - TARGET_HEIGHT) / 2;
    cv::Mat final_rgb = cropped(cv::Rect(start_x, start_y, TARGET_WIDTH, TARGET_HEIGHT));

    // Encode RGB
    std::vector<uchar> jpg_rgb;
    cv::imencode(".jpg", final_rgb, jpg_rgb, {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY});

    // Encode Gray (Y channel only)
    cv::Mat y_channel = yuv(cv::Rect(0, 0, buf->width, buf->height));
    cv::Mat y_resized;
    cv::resize(y_channel, y_resized, cv::Size(), scale, scale, cv::INTER_AREA);
    cv::Mat final_gray = y_resized(cv::Rect(start_x, start_y, TARGET_WIDTH, TARGET_HEIGHT));
    
    std::vector<uchar> jpg_gray;
    cv::imencode(".jpg", final_gray, jpg_gray, {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY});

    {
      std::lock_guard<std::mutex> lock(frame_lock);
      latest_jpeg_rgb = std::move(jpg_rgb);
      latest_jpeg_gray = std::move(jpg_gray);
    }

    auto end_time = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    {
      std::lock_guard<std::mutex> lock(status_lock);
      char buf_text[128];
      snprintf(buf_text, sizeof(buf_text), "JPEG %dx%d | avg %.1f ms | C++ Native", TARGET_WIDTH, TARGET_HEIGHT, elapsed_ms);
      status_text = buf_text;
    }
  }
}

void handle_client(int client_sock) {
  char request[1024];
  int bytes_received = recv(client_sock, request, sizeof(request) - 1, 0);
  if (bytes_received <= 0) {
    close(client_sock);
    return;
  }
  request[bytes_received] = '\0';

  std::string req_str(request);
  std::string response;
  std::vector<uchar> *image_to_send = nullptr;
  std::string content_type = "text/plain";

  if (req_str.find("GET /roadrgb.jpg") != std::string::npos) {
    std::lock_guard<std::mutex> lock(frame_lock);
    if (!latest_jpeg_rgb.empty()) {
      image_to_send = &latest_jpeg_rgb;
      content_type = "image/jpeg";
    }
  } else if (req_str.find("GET /roadgray.jpg") != std::string::npos) {
    std::lock_guard<std::mutex> lock(frame_lock);
    if (!latest_jpeg_gray.empty()) {
      image_to_send = &latest_jpeg_gray;
      content_type = "image/jpeg";
    }
  } else if (req_str.find("GET /status") != std::string::npos) {
    std::lock_guard<std::mutex> lock(status_lock);
    response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: " + std::to_string(status_text.length()) + "\r\n\r\n" + status_text;
  } else {
    std::string body = "<html><body><h1>Laned C++ Native</h1><p><a href='/roadrgb.jpg'>RGB Stream</a></p><p><a href='/roadgray.jpg'>Gray Stream</a></p></body></html>";
    response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: " + std::to_string(body.length()) + "\r\n\r\n" + body;
  }

  if (image_to_send) {
    std::string header = "HTTP/1.1 200 OK\r\nContent-Type: " + content_type + "\r\nContent-Length: " + std::to_string(image_to_send->size()) + "\r\n\r\n";
    send(client_sock, header.c_str(), header.length(), 0);
    send(client_sock, image_to_send->data(), image_to_send->size(), 0);
  } else if (!response.empty()) {
    send(client_sock, response.c_str(), response.length(), 0);
  }

  close(client_sock);
}

void server_thread() {
  int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  int opt = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

  sockaddr_in address;
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(PORT);

  if (bind(server_fd, (sockaddr*)&address, sizeof(address)) < 0) {
    LOGE("laned bind failed");
    return;
  }

  if (listen(server_fd, 10) < 0) {
    LOGE("laned listen failed");
    return;
  }

  LOGW("laned server listening on port %d", PORT);

  while (running) {
    int client_sock = accept(server_fd, nullptr, nullptr);
    if (client_sock < 0) continue;
    std::thread(handle_client, client_sock).detach();
  }
  close(server_fd);
}

int main() {
  prctl(PR_SET_NAME, "laned", 0, 0, 0);

  std::thread cam_t(camera_thread);
  std::thread server_t(server_thread);

  cam_t.join();
  server_t.join();

  return 0;
}
