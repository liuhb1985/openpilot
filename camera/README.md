# Camera 盲区检测系统

## 概述

本系统基于OpenPilot开发，用于车辆周围盲区检测。通过多个摄像头和YOLOv5目标检测模型，实时检测车辆周围的车辆，判断是否存在盲区风险，并通过UDP协议输出检测结果。

## 目录结构

```
camera/
├── c3cam                  # 主要的摄像头盲区检测程序
├── c3cam_8845             # 专为8845平台优化的摄像头盲区检测程序
├── c3cam_amdgpu           # 使用AMD GPU加速的摄像头盲区检测程序
├── c3cam_lane             # 车道线检测程序
├── usb_camera             # USB摄像头程序
├── camera_config.json     # 主配置文件
├── camera_config_8845hs.json # 8845平台专用配置文件
├── usb_camera.json        # USB摄像头配置文件
├── rois.txt               # ROI(感兴趣区域)配置文件
├── yolov5s.onnx           # YOLOv5目标检测模型
├── yolov5s.pt             # YOLOv5模型的PyTorch格式
├── c3cam.cpp              # 主程序源代码
├── c3cam_8845.cpp         # 8845平台版本的源代码
├── c3cam_amdgpu.cpp       # AMD GPU版本的源代码
├── c3cam_lane.cpp         # 车道线检测版本的源代码
├── usb_camera.cpp         # USB摄像头程序源代码
├── json11.cpp/json11.hpp  # JSON解析库
└── find_cam.py            # 查找可用摄像头设备的Python脚本
```

## 系统架构

### 主要组件

1. **摄像头捕获模块**: 使用V4L2接口捕获摄像头视频流
2. **目标检测模块**: 使用YOLOv5模型检测画面中的车辆
3. **ROI过滤模块**: 只关注定义的感兴趣区域内的车辆
4. **盲区判断模块**: 根据摄像头配置判断车辆是否在盲区
5. **通信模块**: 通过UDP广播盲区状态

### 配置文件说明

#### camera_config.json

```json
{
  "debug": false,
  "raw_conf_threshold": 0.5,
  "nms_conf_threshold": 0.5,
  "nms_threshold": 0.5,
  "show_video": true,
  "single_window": true,
  "cameras": [
    {
      "device": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:4:1.0-video-index0",
      "sign": 0,
      "car_detect": 0
    },
    {
      "device": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2.4:1.0-video-index0",
      "sign": 0,
      "car_detect": 1
    }
  ]
}
```

- `debug`: 是否启用调试模式
- `raw_conf_threshold`: 原始置信度阈值
- `nms_conf_threshold`: NMS置信度阈值
- `nms_threshold`: NMS阈值
- `show_video`: 是否显示视频
- `single_window`: 是否在单个窗口显示所有摄像头
- `cameras`: 摄像头配置数组
  - `device`: 摄像头设备路径
  - `sign`: 摄像头检测区域标识 (0=左侧, 1=右侧)
  - `car_detect`: 是否启用车辆检测 (0=不启用, 1=启用)

## 盲区检测原理

### 检测流程

1. **摄像头初始化**: 读取配置文件，初始化所有摄像头设备
2. **视频流捕获**: 使用独立线程捕获每个摄像头的视频流
3. **目标检测**: 对每帧图像使用YOLOv5模型检测车辆
4. **ROI过滤**: 只保留ROI区域内的检测结果
5. **盲区判断**: 根据摄像头的sign值判断是否在盲区
6. **状态输出**: 通过UDP广播盲区状态

### 核心代码逻辑

```cpp
// camera_sign[cam_id]表示摄像头检测区域(0=左侧, 1=右侧)
// camera_car[cam_id]表示该摄像头是否检测到车辆(0=无车, 1=有车)
for(int cam_id = 0; cam_id < camera_car.size(); cam_id++){
    if(camera_sign[cam_id] >= lane_unsafe_tmp.size()){
        continue;
    }
    land_id = camera_sign[cam_id];

    if(camera_car[cam_id] > 0){
        // 如果检测到车辆，标记对应侧为不安全
        lane_unsafe_tmp[land_id] |= 1<<cam_id;
    }
}

// 判断左右两侧是否安全
for(int i=0; i<lane_unsafe_tmp.size(); i++){
    if(lane_unsafe_tmp[i] > 0){
        lane_safe_tmp[i] = false;  // 不安全
    }
    else{
        lane_safe_tmp[i] = true;   // 安全
    }
}
```

## 修改方案

### 1. 一个前摄像头标定两个前盲区

要实现一个前摄像头检测两个前盲区，需要:

#### 修改配置文件

在camera_config.json中添加新的摄像头配置:

```json
{
  "cameras": [
    // 原有配置...
    {
      "device": "/dev/video0",  // 前摄像头设备
      "sign": 2,                // 新增标识，表示前左盲区
      "car_detect": 1
    },
    {
      "device": "/dev/video0",  // 同一个前摄像头设备
      "sign": 3,                // 新增标识，表示前右盲区
      "car_detect": 1
    }
  ]
}
```

#### 修改代码逻辑

在c3cam.cpp中修改盲区判断逻辑:

```cpp
// 增加前盲区状态数组
std::vector<int> front_lane_safe(2, -1);  // 前左、前右盲区状态

// 在lane_check_thread函数中添加前盲区检测逻辑
for(int cam_id = 0; cam_id < camera_car.size(); cam_id++){
    // 原有逻辑保持不变...

    // 添加前盲区检测
    if(camera_sign[cam_id] == 2) {  // 前左盲区
        if(camera_car[cam_id] > 0) {
            front_lane_safe[0] = false;  // 前左不安全
        } else {
            front_lane_safe[0] = true;   // 前左安全
        }
    } else if(camera_sign[cam_id] == 3) {  // 前右盲区
        if(camera_car[cam_id] > 0) {
            front_lane_safe[1] = false;  // 前右不安全
        } else {
            front_lane_safe[1] = true;   // 前右安全
        }
    }
}
```

#### 修改ROI定义

在主函数中为前摄像头定义两个不同的ROI区域:

```cpp
// 为前摄像头定义两个ROI区域(前左和前右)
camera_rois.resize(cam_max_num);
for (int i = 0; i < cam_max_num; i++) {
    if (camera_sign[i] == 2) {  // 前左盲区ROI
        camera_rois[i].polygon = {cv::Point(100, 100), cv::Point(300, 100),
                                  cv::Point(300, 300), cv::Point(100, 300)};
    } else if (camera_sign[i] == 3) {  // 前右盲区ROI
        camera_rois[i].polygon = {cv::Point(340, 100), cv::Point(540, 100),
                                  cv::Point(540, 300), cv::Point(340, 300)};
    } else {
        // 原有ROI定义
        camera_rois[i].polygon = {cv::Point(100,50), cv::Point(540,50),
                                  cv::Point(540,430), cv::Point(100,430)};
    }
}
```

### 2. 添加两个后盲区

要添加两个后盲区检测，需要:

#### 修改配置文件

在camera_config.json中添加后摄像头配置:

```json
{
  "cameras": [
    // 原有配置...
    {
      "device": "/dev/video1",  // 后摄像头设备
      "sign": 4,                // 新增标识，表示后左盲区
      "car_detect": 1
    },
    {
      "device": "/dev/video1",  // 后摄像头设备
      "sign": 5,                // 新增标识，表示后右盲区
      "car_detect": 1
    }
  ]
}
```

#### 修改代码逻辑

在c3cam.cpp中扩展盲区检测:

```cpp
// 增加后盲区状态数组
std::vector<int> rear_lane_safe(2, -1);  // 后左、后右盲区状态

// 在lane_check_thread函数中添加后盲区检测逻辑
for(int cam_id = 0; cam_id < camera_car.size(); cam_id++){
    // 原有逻辑保持不变...

    // 添加后盲区检测
    if(camera_sign[cam_id] == 4) {  // 后左盲区
        if(camera_car[cam_id] > 0) {
            rear_lane_safe[0] = false;  // 后左不安全
        } else {
            rear_lane_safe[0] = true;   // 后左安全
        }
    } else if(camera_sign[cam_id] == 5) {  // 后右盲区
        if(camera_car[cam_id] > 0) {
            rear_lane_safe[1] = false;  // 后右不安全
        } else {
            rear_lane_safe[1] = true;   // 后右安全
        }
    }
}
```

#### 修改ROI定义

为后摄像头定义两个不同的ROI区域:

```cpp
// 为后摄像头定义两个ROI区域(后左和后右)
camera_rois.resize(cam_max_num);
for (int i = 0; i < cam_max_num; i++) {
    if (camera_sign[i] == 2) {  // 前左盲区ROI
        camera_rois[i].polygon = {cv::Point(100, 100), cv::Point(300, 100),
                                  cv::Point(300, 300), cv::Point(100, 300)};
    } else if (camera_sign[i] == 3) {  // 前右盲区ROI
        camera_rois[i].polygon = {cv::Point(340, 100), cv::Point(540, 100),
                                  cv::Point(540, 300), cv::Point(340, 300)};
    } else if (camera_sign[i] == 4) {  // 后左盲区ROI
        camera_rois[i].polygon = {cv::Point(100, 150), cv::Point(250, 150),
                                  cv::Point(250, 400), cv::Point(100, 400)};
    } else if (camera_sign[i] == 5) {  // 后右盲区ROI
        camera_rois[i].polygon = {cv::Point(390, 150), cv::Point(540, 150),
                                  cv::Point(540, 400), cv::Point(390, 400)};
    } else {
        // 原有ROI定义
        camera_rois[i].polygon = {cv::Point(100,50), cv::Point(540,50),
                                  cv::Point(540,430), cv::Point(100,430)};
    }
}
```

#### 修改UDP输出

在UDP通信部分添加前后盲区状态输出:

```cpp
Json resp = Json::object{
    {"resp", "ok"},
    {"timeout", false},
    {"ip", local_ip},
    {"port", LOCAL_RECV_PORT},
    {"left_blind", lane_safe[0]?false:true},      // 左侧盲区
    {"right_blind", lane_safe[1]?false:true},     // 右侧盲区
    {"front_left_blind", front_lane_safe[0]?false:true},   // 前左盲区
    {"front_right_blind", front_lane_safe[1]?false:true},  // 前右盲区
    {"rear_left_blind", rear_lane_safe[0]?false:true},     // 后左盲区
    {"rear_right_blind", rear_lane_safe[1]?false:true}     // 后右盲区
};
```

## 注意事项

1. **ROI区域调整**: ROI区域的定义需要根据实际摄像头安装位置和视角进行调整
2. **设备路径**: 确保摄像头设备路径正确
3. **检测阈值**: 可能需要调整YOLO检测的置信度阈值以适应不同的检测环境
4. **通信协议**: UDP输出格式需要与接收端保持一致
5. **性能优化**: 在资源受限的设备上，可能需要调整检测频率或降低图像分辨率

## 启动方式

```bash
# 启动主程序
./c3cam

# 启动8845平台版本
./c3cam_8845

# 启动AMD GPU加速版本
./c3cam_amdgpu

# 启动车道线检测版本
./c3cam_lane

# 启动USB摄像头程序
./usb_camera
```

## 总结

通过以上修改，可以实现:
1. 一个前摄像头检测两个前盲区(前左和前右)
2. 添加两个后盲区检测(后左和后右)

这样的修改将使盲区检测系统更加全面，能够检测车辆四周的盲区，提高驾驶安全性。