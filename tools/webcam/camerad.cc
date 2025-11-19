#include "camera.h"
#include "camerad.h"

#include <vector>

#define USE_ROADCAMERASTATE
#define USE_WIDEROADCAMERASTATE
#define CAM_WIDTH 2592//1920
#define CAM_HEIGHT 1944//1080
#define CAM_FPS 20

const char *PATH_VIDEOS = "/home/op/videos/"; // Save record videos.

int mac();

// 检查是否为视频设备文件
int is_video_device(const char *name) {
    return strncmp(name, "video", 5) == 0;
}

// 获取视频设备信息
const char * query_device(const char *device_path, char *result) {
    int fd = open(device_path, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "无法打开设备 %s: %s\n", device_path, strerror(errno));
        return "";
    }

    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        fprintf(stderr, "无法查询设备 %s 的能力: %s\n", device_path, strerror(errno));
        close(fd);
        return "";
    }
    close(fd);

    snprintf(result, 255, "%s", cap.bus_info);
    return (const char*)result;
}
/*
const char *get_device(const char *addr, char *device)
{
     DIR *dir;
    struct dirent *entry;
    char device_path[500] = {0};

    dir = opendir("/dev");
    if (!dir) {
        fprintf(stderr, "无法打开 /dev 目录: %s\n", strerror(errno));
        return "";
    }

    while ((entry = readdir(dir)) != NULL) {
        if (is_video_device(entry->d_name)) {
            if (strlen(entry->d_name) > (255 - 5)) {
                fprintf(stderr, "警告: 设备名 %s 过长，跳过\n", entry->d_name);
                continue;
            }

            static int falg = strlen("video");
            int n = atoi(&entry->d_name[falg]);
            // printf("device name:%s %d\n", entry->d_name, n);
            snprintf(device_path, sizeof(device_path), "/dev/%s", entry->d_name);
            char buf[256] = {0};
            query_device(device_path, buf);
            printf("%s\n", buf);

            if (0==strcmp(addr, buf) && n%2==0)
            {
                printf("ok\n");
                snprintf(device, 510, "%s", device_path);
                return (const char*)device;
            }

        }
    }

    closedir(dir);
    return "";
}
*/
void camerad::camera_runner() {
  //cl_device_id device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  //cl_device_type device_type = CL_DEVICE_TYPE_GPU | CL_DEVICE_TYPE_CPU;
  cl_device_type device_type = CL_DEVICE_TYPE_GPU;
  cl_device_id device_id = cl_get_device_id(device_type);
  cl_context context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));
  VisionIpcServer vipc_server("camerad", device_id, context);

  for (camera *cam : m_cameras) {
    vipc_server.create_buffers(cam->get_stream_type(),20, cam->width(), cam->height()); //false,
  }
  vipc_server.start_listener();
  RateKeeper rk("roadCameraState", 20);

  uint32_t frame_id = 1;
  while (!m_do_exit) {
    for (camera *cam : m_cameras) {
          cam->send_yuv(frame_id, vipc_server);
      }

    frame_id++;
    rk.keepTime();
  }
}

void camerad::run() {
  //if (mac() != 0) {
    // return;
  //}

#ifdef USE_ROADCAMERASTATE
  char device_path0[512] = {0};
#endif
#ifdef USE_ROADCAMERASTATE
  char device_path1[512] = {0};
#endif
  //get_device("usb-0000:03:00.4-3", device_path0);
  // get_device("usb-0000:04:00.3-2", device_path0);
  //std::cout << device_path0 <<  std::endl;
  //get_device("usb-0000:03:00.3-4", device_path1);
  //std::cout << device_path1 <<  std::endl;

#ifdef USE_ROADCAMERASTATE
  if (strlen(device_path0) == 0)
  {
      //std::cerr << "Error finding video " << "usb-0000:04:00.3-2.2" << std::endl;
      strcpy(device_path0, "/dev/video0");
      // return;
  }
#endif

#ifdef USE_ROADCAMERASTATE
  if (strlen(device_path1) == 0)
  {
      //std::cerr << "Error finding video " << "usb-0000:04:00.3-2.3" << std::endl;
      strcpy(device_path1, "/dev/video2");
      // return;
  }
#endif

#ifdef USE_ROADCAMERASTATE
  const char* device0 = device_path0;//"/dev/video0";
  int width = CAM_WIDTH;
  int height = CAM_HEIGHT;
  std::string output_prefix = PATH_VIDEOS; // PC has a large hard drive capacity, so you can use it as a dashcam.
  camera *cam_road = new camera(device0, "roadCameraState", width, height, CAM_FPS, output_prefix);
  m_cameras.push_back(cam_road);
#endif

#ifdef USE_WIDEROADCAMERASTATE
  const char* device1 = device_path1; //"/dev/video2";
  width = CAM_WIDTH;
  height = CAM_HEIGHT;
  camera *cam_wide = new camera(device1, "wideRoadCameraState", width, height,  CAM_FPS, output_prefix);
  m_cameras.push_back(cam_wide);
#endif

  for (camera *cam : m_cameras) {
      cam->run();
  }

  std::thread th_cam(&camerad::camera_runner, this);
  th_cam.join();
}

/*
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>

int mac() {
  struct ifreq ifr;
  int _fd = socket(AF_INET, SOCK_DGRAM, 0);

  //strcpy(ifr.ifr_name, "eno1");
  strcpy(ifr.ifr_name, "enp2s0");
  if (ioctl(_fd, SIOCGIFHWADDR, &ifr) == -1) {
      perror("Failed to get");
      close(_fd);
      return 1;
  }

  unsigned char *mac = (unsigned char *)ifr.ifr_hwaddr.sa_data;
  char tmp_mac[64] = {0};
  sprintf(tmp_mac , "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  printf("%s ",tmp_mac);
  if (strcmp("00e04d681709", tmp_mac)==0 ||
  strcmp("001696ed01ed", tmp_mac)==0 ||
  strcmp("00e04d681709", tmp_mac)==0 ||
  strcmp("00f1f5363e64", tmp_mac)==0) {
    close(_fd);

    return 0;
  }

  //printf("%s\n", tmp_mac);
  printf("Too bad...");
  close(_fd);
  //tmp_mac[4] = '\0';
  //printf("%s\n", tmp_mac);
  return 1;
}
*/
