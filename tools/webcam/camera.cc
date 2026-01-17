#include "camera.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
}

#include <libavutil/dict.h>
#include <immintrin.h>

#define COUNT_VIDEOS (60*7)


// decode_jpeg output NV12
bool decode_jpeg(const void* mjpeg_data, size_t mjpeg_size, std::vector<uint8_t>& yuv, std::vector<uint8_t>& nv12, int& width, int& height, tjhandle handle) {
    if (!handle) {
        std::cerr << "Invalid JPEG handle" << std::endl;
        return false;
    }

    int jpegSubsamp = 0;
    if (tjDecompressHeader2(handle, (unsigned char*)mjpeg_data, mjpeg_size, &width, &height, &jpegSubsamp) < 0) {
        std::cerr << "Failed to read JPEG header: " << tjGetErrorStr2(handle) << std::endl;
        return false;
    }

    if (width <= 0 || height <= 0) {
        std::cerr << "Invalid JPEG dimensions: " << width << "x" << height << std::endl;
        return false;
    }

    if (!handle || !mjpeg_data || mjpeg_size == 0) {
        std::cerr << "Failed to mjpeg_data JPEG error" << std::endl;
        return false;
    }

    int y_size = width * height;
    if (jpegSubsamp == TJSAMP_422) {
        // printf("TJSAMP_422 \n");
         // 计算大小
        int uv_size = (jpegSubsamp == TJSAMP_422) ? (width / 2) * height : (width / 2) * (height / 2);

        // 分配缓冲区
        unsigned char *yuv422 = yuv.data(); // YUV422 平面
        unsigned char *nv = nv12.data();     // NV12 缓冲区
        uint8_t *y_ptr = nv;
        uint8_t *uv_ptr = nv + y_size;
        uint8_t *u_plane = yuv422 + y_size;
        uint8_t *v_plane = u_plane + uv_size;

        // 解码 JPEG 到 YUV422 平面
        unsigned char *planes[3] = { yuv422, u_plane, v_plane };
        int strides[3] = { width, width / 2, width / 2 };
        if (tjDecompressToYUVPlanes(handle, (unsigned char*)mjpeg_data, mjpeg_size, planes, width, strides, height, 0) != 0) {
            printf("Error decoding JPEG to YUV planes: %s\n", tjGetErrorStr2(handle));
            return false;
        }

        // 复制 Y 平面
        memcpy(y_ptr, yuv422, y_size);

        // 转换 U 和 V 到 NV12 UV 平面（4:2:2 到 4:2:0）
        int nv12_uv_size = (width / 2) * (height / 2);
        for (int i = 0; i < nv12_uv_size - 15; i += 16) {
            int src_idx = (i / (width / 2)) * 2 * (width / 2) + (i % (width / 2)); // 取偶数行
            __m128i u = _mm_loadu_si128((__m128i*)(u_plane + src_idx));
            __m128i v = _mm_loadu_si128((__m128i*)(v_plane + src_idx));
            __m128i uv_lo = _mm_unpacklo_epi8(u, v);
            __m128i uv_hi = _mm_unpackhi_epi8(u, v);
            _mm_storeu_si128((__m128i*)(uv_ptr + 2 * i), uv_lo);
            _mm_storeu_si128((__m128i*)(uv_ptr + 2 * i + 16), uv_hi);
        }
        // 处理剩余像素
        for (int i = (nv12_uv_size / 16) * 16; i < nv12_uv_size; i++) {
            int src_idx = (i / (width / 2)) * 2 * (width / 2) + (i % (width / 2));
            uv_ptr[2 * i] = u_plane[src_idx];
            uv_ptr[2 * i + 1] = v_plane[src_idx];
        }
    } else {
        printf("tjDecompressToYUV2 \n");
        // 直接解码到 YUV420P（接近 NV12）
        if (tjDecompressToYUV2(handle, (unsigned char*)mjpeg_data, mjpeg_size, yuv.data(), width, 1, height, TJFLAG_ACCURATEDCT ) < 0) {//TJFLAG_FASTDCT| TJFLAG_FASTUPSAMPLE
            std::cerr << "Failed to decompress JPEG: " << tjGetErrorStr2(handle) << std::endl;
            nv12.resize(0);
            return false;
        }

        if (nv12.size() == 0) {
            int uv_size = width * height / 2;
            std::cerr << "start alloc " << y_size + uv_size << std::endl;
            nv12.resize(y_size + uv_size);
        }

        const uint8_t* u_ptr = yuv.data() + y_size;
        const uint8_t* v_ptr = u_ptr + (y_size / 4);
        uint8_t* uv_ptr = nv12.data() + y_size;

         memcpy(nv12.data(), yuv.data(), y_size);
        int uv_size = y_size / 4;

         for (int i = 0; i < uv_size; i += 16) {
             __m128i u = _mm_loadu_si128((__m128i*)(u_ptr + i)); // 加载 16 个 U
             __m128i v = _mm_loadu_si128((__m128i*)(v_ptr + i)); // 加载 16 个 V
             __m128i uv_lo = _mm_unpacklo_epi8(u, v);  // [U0,V0,...,U7,V7]
             __m128i uv_hi = _mm_unpackhi_epi8(u, v);  // [U8,V8,...,U15,V15]
             _mm_storeu_si128((__m128i*)(uv_ptr + 2 * i), uv_lo);
             _mm_storeu_si128((__m128i*)(uv_ptr + 2 * i + 16), uv_hi);
         }

        for (size_t i = (uv_size / 16) * 16; i < uv_size; ++i) {
            uv_ptr[2 * i] = u_ptr[i];
            uv_ptr[2 * i + 1] = v_ptr[i];
        }
    }

    return true;
}

camera::camera(const char* device, const char* type, int width, int height, int fps, const std::string& output_prefix)
      : m_device(device), m_type(type), m_width(width), m_height(height),
      m_fps(fps), m_output_prefix(output_prefix),m_pm(NULL),
      m_current_exposure(0) {

    if (m_type == "roadCameraState") {
        m_pm = new PubMaster({"roadCameraState"});
        m_type_vision = VISION_STREAM_ROAD;
    } else if (m_type == "driverCameraState") {
        m_pm = new PubMaster({"driverCameraState"});
        m_type_vision = VISION_STREAM_DRIVER;
    } else {
        m_pm = new PubMaster({"wideRoadCameraState"});
        m_type_vision = VISION_STREAM_WIDE_ROAD;
    }
}

camera::~camera() {
    if (m_pm) {
        delete m_pm;
        m_pm = nullptr;
    }
}

// open camera
int camera::open_camera(const char* device) {
    m_fd = open(device, O_RDWR);

    if (m_fd == -1) {
        std::cerr << "Failed to open device: " << device << std::endl;
        return -1;
    }

    return m_fd;
}

// set camera format
bool camera::set_camera_format(int fd, int width, int height, uint32_t pixel_format) {
    v4l2_format format = {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = pixel_format;
    format.fmt.pix.field = V4L2_FIELD_ANY;

    if (ioctl(fd, VIDIOC_S_FMT, &format) == -1) {
        std::cerr << "Failed to set format" << std::endl;
        return false;
    }
    if (format.fmt.pix.pixelformat != pixel_format) {
        std::cerr << "The camera does not support the requested format." << std::endl;
        return false;
    }
    return true;
}

// set fps
bool camera::set_frame_rate(int fd, int fps) {
    v4l2_streamparm parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_PARM, &parm) == -1) {
        std::cerr << "Failed to get param" << std::endl;
        return false;
    }
    if (!(parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
        std::cerr << "Failed to set fps" << std::endl;
        return false;
    }
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    if (ioctl(fd, VIDIOC_S_PARM, &parm) == -1) {
        std::cerr << "Failed to set fps as " << fps << " FPS" << std::endl;
        return false;
    }
    if (ioctl(fd, VIDIOC_G_PARM, &parm) != -1) {
        int set_fps = parm.parm.capture.timeperframe.denominator / parm.parm.capture.timeperframe.numerator;
        std::cout << "current " << set_fps << " FPS" << std::endl;
    }

    return true;
}

bool camera::set_exposure(int fd, int value) {
    v4l2_control ctrl = {};
    ctrl.id = V4L2_EXPOSURE_AUTO;
    ctrl.value = V4L2_EXPOSURE_MANUAL;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1) {
        std::cerr << "Failed to set exposure" << std::endl;
//        return false;
    }

    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = value;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1){
        std::cerr << "Failed to set exposure time" << std::endl;
        return false;
    }

    ctrl.id = V4L2_CID_GAIN;
    ctrl.value = 64;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1){
        std::cerr << "Failed to set gain" << std::endl;
        return false;
    }

    return true;
}

// init buffer
bool camera::init_buffer(int fd) {
    if (!init_mmap(fd, m_buffers, m_buffer_sizes)) {
        close(fd);
        return false;
    }
    return true;
}

// init mmap
bool camera::init_mmap(int fd, std::vector<void*>& buffers, std::vector<size_t>& buffer_sizes) {
    v4l2_requestbuffers req = {};
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    req.count = 4;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        std::cerr << "Failed to request buff" << std::endl;
        return false;
    }

    buffers.resize(req.count);
    buffer_sizes.resize(req.count);

    for (size_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            std::cerr << "Query buffer failed." << std::endl;
            return false;
        }

        buffers[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (buffers[i] == MAP_FAILED) {
            std::cerr << "map buffer failed." << std::endl;
            return false;
        }
        buffer_sizes[i] = buf.length;
    }
    return true;
}

// start capture
bool camera::start_capture(int fd) {
    for (size_t i = 0; i < 4; ++i) {
        v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            std::cerr << "Queue buffer failed." << std::endl;
            return false;
        }
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        std::cerr << "capture failed." << std::endl;
        return false;
    }
    return true;
}

bool camera::read_frame(std::vector<void*>& buffers, std::vector<size_t>& buffer_sizes, void*& out_data, size_t& out_size) {
    v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(m_fd, VIDIOC_DQBUF, &buf) == -1) {
        std::cerr << "Dequeue buffer failed. " << m_fd << std::endl;
        usleep(10);
        return false;
    }
    out_data = buffers[buf.index];
    out_size = buf.bytesused;
    if (ioctl(m_fd, VIDIOC_QBUF, &buf) == -1) {
        std::cerr << "Failed to requeue the buffer." << std::endl;
        return false;
    }

    return true;
}

// reading loop from camera
void camera::reading_loop(int fd) {
    auto start = nanos_since_boot();
    int count = 0;
    int count_empty = 0;
    m_jpeg_handle = tjInitDecompress();
    m_yuv_data.resize(m_width*m_height + m_width*m_height/2);

    std::vector<uint8_t> yuv420_data;
    std::vector<uint8_t> nv12_data;
    yuv420_data.resize(m_width*m_height + m_width*m_height);
    nv12_data.resize(m_width*m_height + m_width*m_height/2);

    while (!m_do_exit) {
        void* mjpeg_data = nullptr;
        size_t mjpeg_size = 0;

        if (!read_frame(m_buffers, m_buffer_sizes, mjpeg_data, mjpeg_size)) {
            continue;
        }

        auto frame_sof = nanos_since_boot();
        int yuv_width, yuv_height;

        if (!decode_jpeg(mjpeg_data, mjpeg_size, yuv420_data, nv12_data, yuv_width, yuv_height, m_jpeg_handle)) {
            std::cerr << "decode jpeg failed." << std::endl;
            count_empty++;
            printf("%s count empty=%d mjpeg_size=%ld \n", m_type.c_str(), count_empty, mjpeg_size);
            if (count_empty > 30) {
                exit(-2);
            }

            continue;
        }
        count_empty = 0;

        // update m_yuv_data
        {
            std::lock_guard<std::mutex> lk(m_mutex);
            m_frame_sof = frame_sof;
            if (nv12_data.size() != m_yuv_data.size())
            {
                std::cerr << "m_yuv_data.resize. " << std::endl;
                m_yuv_data.resize(nv12_data.size());
            }

            memcpy(m_yuv_data.data(), nv12_data.data(), nv12_data.size());

            m_frame_eof = nanos_since_boot();
        }

        count ++;
        auto tm = nanos_since_boot() - start;
        if (tm/(1000*1000) >= 1000) {
            printf("%s fps = %d\n", m_type.c_str(), count);
            count = 0;
            start = nanos_since_boot();
        }

        if (m_output_prefix != "") {
            std::vector<uint8_t> mjpeg_data_copy(mjpeg_size);
            memcpy(mjpeg_data_copy.data(), mjpeg_data, mjpeg_size);
            m_frame_queue.push(std::move(mjpeg_data_copy));
        }

        usleep(100);
    }
}

// Send yuv
void camera::send_yuv(uint32_t frame_id, VisionIpcServer &vipc_server) {
  auto cur_yuv_buf = vipc_server.get_buffer(m_type_vision);
  cur_yuv_buf->set_frame_id(frame_id);

  {
    static int count_err = 0;
    std::lock_guard<std::mutex> lk(m_mutex);
    if (!m_yuv_data.empty()) {
        memcpy(cur_yuv_buf->addr, m_yuv_data.data(), m_yuv_data.size());
        count_err = 0;
    } else {
        std::cerr << "Error: send_yuv, m_yuv_data.empty() " << m_type << std::endl;
        count_err++;
        if (count_err > 30)
          exit(-2);
        return;
    }
  }

  auto timestamp_sof = (uint64_t)(frame_id*0.05*1000000000);//m_frame_sof;//
  VisionIpcBufExtra extra = {frame_id, timestamp_sof, timestamp_sof};//m_frame_eof
  vipc_server.send(cur_yuv_buf, &extra, false);

  MessageBuilder msg;
  if (VISION_STREAM_ROAD == m_type_vision) {
    auto cam_s = msg.initEvent().initRoadCameraState();
    cam_s.setFrameId(frame_id);
    std::vector<float> data = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cam_s.setTransform(kj::ArrayPtr<const float>(data.data(), data.size()));
  } else if (VISION_STREAM_WIDE_ROAD == m_type_vision) {
    auto cam_s = msg.initEvent().initWideRoadCameraState();
    cam_s.setFrameId(frame_id);
    std::vector<float> data = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cam_s.setTransform(kj::ArrayPtr<const float>(data.data(), data.size()));
  }else {
    auto cam_s = msg.initEvent().initDriverCameraState();
    cam_s.setFrameId(frame_id);
    std::vector<float> data = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cam_s.setTransform(kj::ArrayPtr<const float>(data.data(), data.size()));
  }

  assert(m_pm != NULL);
  m_pm->send(m_type.c_str(), msg);
}

void camera::saving_loop() {
    if (m_output_prefix == "") {
        return;
    }

    auto last_switch_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    AVFormatContext* fmt_ctx = nullptr;
    AVPacket* pkt = nullptr;
    AVStream* stream = nullptr;
    AVCodecContext* codec_ctx = nullptr;

    // Initialize FFmpeg
    auto init_ffmpeg = [&](const std::string& filename) {
        // Clean up existing context
        if (fmt_ctx) {
            av_write_trailer(fmt_ctx);
            avio_closep(&fmt_ctx->pb);
            avformat_free_context(fmt_ctx);
            fmt_ctx = nullptr;
        }
        if (codec_ctx) {
            avcodec_free_context(&codec_ctx);
        }

        // Create output format context (using MP4 container)
        if (avformat_alloc_output_context2(&fmt_ctx, nullptr, "mp4", filename.c_str()) < 0) {
            std::cerr << "Failed to create " << filename << " file" << std::endl;
            return false;
        }

        // Find MJPEG encoder
        const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
        if (!codec) {
            std::cerr << "MJPEG encoder not found" << std::endl;
            return false;
        }

        // Create stream
        stream = avformat_new_stream(fmt_ctx, nullptr);
        if (!stream) {
            std::cerr << "Failed to create stream" << std::endl;
            return false;
        }

        // Create codec context
        codec_ctx = avcodec_alloc_context3(codec);
        if (!codec_ctx) {
            std::cerr << "Failed to allocate codec context" << std::endl;
            return false;
        }

        // Configure codec parameters
        codec_ctx->width = m_width;
        codec_ctx->height = m_height;
        codec_ctx->time_base = {1, 30}; // 30 FPS
        codec_ctx->pix_fmt = AV_PIX_FMT_YUVJ420P; // MJPEG uses YUVJ420P
        codec_ctx->framerate = {30, 1}; // Explicitly set framerate

        // Set JPEG quality
        AVDictionary* opts = nullptr;
        av_dict_set(&opts, "q:v", "1", 0); // JPEG quality (1-31, 5 is high compression with good quality)

        // Open encoder
        if (avcodec_open2(codec_ctx, codec, &opts) < 0) {
            std::cerr << "Failed to open MJPEG encoder" << std::endl;
            av_dict_free(&opts);
            avcodec_free_context(&codec_ctx);
            return false;
        }
        av_dict_free(&opts);

        // Set stream parameters
        stream->codecpar->codec_id = AV_CODEC_ID_MJPEG;
        stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        stream->codecpar->width = m_width;
        stream->codecpar->height = m_height;
        stream->time_base = {1, 30};
        stream->r_frame_rate = {30, 1}; // Explicitly set stream framerate

        // Open output file
        if (avio_open(&fmt_ctx->pb, filename.c_str(), AVIO_FLAG_WRITE) < 0) {
            std::cerr << "Failed to create file " << filename << std::endl;
            avcodec_free_context(&codec_ctx);
            return false;
        }

        // Write file header
        if (avformat_write_header(fmt_ctx, nullptr) < 0) {
            std::cerr << "Failed to write header for " << filename << std::endl;
            avcodec_free_context(&codec_ctx);
            return false;
        }

        std::cout << "Successfully initialized FFmpeg for " << filename << std::endl;
        return true;
    };

    // Delete old video files
    auto manage_video_files = [&]() {
        const char* folder = m_output_prefix.c_str();
        DIR* dir = opendir(folder);
        if (!dir) {
            std::cerr << "Failed to open directory" << std::endl;
            return;
        }

        std::vector<std::pair<std::string, time_t>> files;
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string fname = entry->d_name;
            if (fname != "." && fname != "..") {
                std::string full_path = folder + fname;
                struct stat file_stat;
                if (stat(full_path.c_str(), &file_stat) != 0) {
                    std::cerr << "Failed to stat " << full_path << ": " << strerror(errno) << std::endl;
                    continue;
                }
                files.emplace_back(full_path, file_stat.st_mtime);
            }
        }
        closedir(dir);

        if (files.size() > COUNT_VIDEOS) {
            std::sort(files.begin(), files.end(),
              [](const auto& a, const auto& b) {
                  return a.second < b.second;
              });

            while (files.size() > COUNT_VIDEOS) {
                const std::string& oldest_file = files.front().first;
                if (remove(oldest_file.c_str()) == 0) {
                    std::cout << "Deleted old video file: " << oldest_file << std::endl;
                } else {
                    std::cerr << "Failed to delete " << oldest_file << ": " << strerror(errno) << std::endl;
                    auto ite = files.begin()+1;
                    if (files.end() != ite) {
                       remove(ite->first.c_str());
                       break;
                    }
                }
                files.erase(files.begin());
            }
        }
    };

    // Initialize packet
    pkt = av_packet_alloc();
    if (!pkt) {
        std::cerr << "Failed to allocate packet" << std::endl;
        return;
    }

    // Initialize video file
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm* tm = std::localtime(&tt);
    char filename[50];
    snprintf(filename, sizeof(filename), "%s%s_%04d%02d%02d_%02d%02d.mp4",
             m_output_prefix.c_str(), m_type.c_str(), tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min);
    if (!init_ffmpeg(filename)) {
        av_packet_free(&pkt);
        return;
    }

    // Start file management thread
    std::thread file_manager([&]() {
        while (!m_do_exit) {
            manage_video_files();
            std::this_thread::sleep_for(std::chrono::seconds(10)); // Run every 10 seconds
        }
    });
    file_manager.detach();

    const std::chrono::milliseconds frame_duration(1000 / 20); // 20 FPS
    auto next_frame_time = std::chrono::steady_clock::now();

    while (!m_do_exit) {
        // Fetch MJPEG data
        std::vector<uint8_t> mjpeg_data_copy;
        if (!m_frame_queue.pop(mjpeg_data_copy)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_switch_time).count();
        if (elapsed >= 60) {
            if (frame_count % 30 == 0) { // Log every second
                std::cout << "After " << frame_count << " frames, creating new video" << std::endl;
            }
            now = std::chrono::system_clock::now();
            tt = std::chrono::system_clock::to_time_t(now);
            tm = std::localtime(&tt);
            snprintf(filename, sizeof(filename), "%s%s_%04d%02d%02d_%02d%02d.mp4",
                     m_output_prefix.c_str(), m_type.c_str(), tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
                     tm->tm_hour, tm->tm_min);
            if (!init_ffmpeg(filename)) {
                break;
            }
            last_switch_time = current_time;
            frame_count = 0;
        }

        // Write MJPEG frame
        av_packet_unref(pkt);
        pkt->data = mjpeg_data_copy.data();
        pkt->size = mjpeg_data_copy.size();
        pkt->stream_index = stream->index;
        pkt->pts = av_rescale_q(frame_count, {1, 20}, stream->time_base);
        pkt->dts = pkt->pts;

        if (av_interleaved_write_frame(fmt_ctx, pkt) < 0) {
            std::cerr << "Failed to write frame" << std::endl;
            break;
        }
        frame_count++;

        next_frame_time += frame_duration;
        std::this_thread::sleep_until(next_frame_time);
    }

    // Clean up resources
    av_write_trailer(fmt_ctx);
    av_packet_free(&pkt);
    avio_closep(&fmt_ctx->pb);
    avcodec_free_context(&codec_ctx);
    avformat_free_context(fmt_ctx);
}

// close camera
void camera::close_camera() {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(m_fd, VIDIOC_STREAMOFF, &type);
    for (size_t i = 0; i < m_buffers.size(); ++i) {
        munmap(m_buffers[i], m_buffer_sizes[i]);
    }

    close(m_fd);
}

bool camera::init_cam() {
  int fd = open_camera(m_device);
    if (fd < 0) {
        return false;
    }

    if (!set_camera_format(fd, m_width, m_height, V4L2_PIX_FMT_MJPEG)) {
        close_camera();
        return false;
    }

    if (!set_frame_rate(fd, m_fps)) {
        close_camera();
        return false;
    }

    if (!init_buffer(fd)) {
        close_camera();
        return false;
    }

    if (!start_capture(fd)) {
        close_camera();
        return false;
    }

    return true;
}

bool camera::run() {
    init_cam();

    // reading_loop
    std::vector<std::thread> threads;
    threads.emplace_back(&camera::reading_loop, this, std::ref(m_fd));
    threads.emplace_back(&camera::saving_loop, this);
    for (auto& t : threads) {
      t.detach();
    }

    return true;
}

double camera::calculate_y_average (std::vector<uint8_t> &yuv_data, int width, int height) {
    const uint8_t * y_ptr = yuv_data.data();
    int y_size = width*height;
    double sum = 0.0;
    for (int i=0; i < y_size; i+=5) {
        sum += y_ptr[i];
        sum += y_ptr[i+1];
        sum += y_ptr[i+2];
        sum += y_ptr[i+3];
        sum += y_ptr[i+4];
    }

    return sum/y_size;
}

void camera::adjust_exposure(int fd, int min, int max) {
    return;
    double y_avg = calculate_y_average(m_yuv_data, m_width, m_height);
    int step = 1;

    if (y_avg < 85.0) {
        m_current_exposure += step;
        if (m_current_exposure > max) {
            m_current_exposure -= step;
        }
        set_exposure(fd, m_current_exposure);
    } else if (y_avg > 150.0) {
        m_current_exposure -= step;
        if (m_current_exposure < min) {
            m_current_exposure += step;
        }
        set_exposure(fd, m_current_exposure);
    }
}
