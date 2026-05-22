import numpy as np
import cv2
import threading
import queue
import os

# 强制禁用 OpenCV OpenCL 后端加速，消除现代显卡驱动（如 RDNA3 等）中的 busy-wait GPU 飙升异常
cv2.ocl.setUseOpenCL(False)


def bgr_to_nv12_bytes(bgr):
    """Convert a BGR ndarray to NV12 bytes using optimized OpenCV/NumPy.
    NV12 format: Y plane followed by interleaved UV plane.
    """
    h, w = bgr.shape[:2]
    # 1. Convert BGR to YUV I420 (Y plane followed by U and V planes)
    yuv_i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)

    # 2. Extract Y plane
    y_plane = yuv_i420[:h, :]

    # 3. Extract U and V planes and interleave them for NV12
    # I420 layout: Y(h*w), U(h*w/4), V(h*w/4)
    u_plane = yuv_i420[h : h + h // 4, :].reshape(-1)
    v_plane = yuv_i420[h + h // 4 :, :].reshape(-1)

    # Use numpy to interleave U and V efficiently
    uv_interleaved = np.empty(h * w // 2, dtype=np.uint8)
    uv_interleaved[0::2] = u_plane
    uv_interleaved[1::2] = v_plane

    return y_plane.tobytes() + uv_interleaved.tobytes()

class Camera:
    def __init__(self, cam_type_state, stream_type, camera_id):
        try:
            camera_id = int(camera_id)
        except ValueError: # allow strings, ex: /dev/video0
            pass
        self.cam_type_state = cam_type_state
        self.stream_type = stream_type
        self.cur_frame_id = 0

        import av
        self.container = av.open(camera_id)
        assert self.container.streams.video, f"Can't open video stream for camera {camera_id}"
        self.video_stream = self.container.streams.video[0]
        self.W = self.video_stream.codec_context.width
        self.H = self.video_stream.codec_context.height

    @classmethod
    def bgr2nv12(self, bgr):
        return bgr_to_nv12_bytes(bgr)

    def read_frames(self):
        for frame in self.container.decode(self.video_stream):
            img = frame.to_rgb().to_ndarray()[:,:, ::-1] # convert to bgr24
            yuv = Camera.bgr2nv12(img)
            yield yuv
        self.container.close()

class CameraMJPG:
    def __init__(self, cam_type_state, stream_type, camera_id, num_workers=None, max_queue_size=10, use_processes=False, width=1920, height=1080):
        try:
            camera_id = int(camera_id)
        except ValueError:
            pass

        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise OSError(f"无法打开摄像头设备 {camera_id}")

        self._configure_camera_format("MJPG", width, height)
        actual_format = self._get_current_format()
        print("数据格式: ", actual_format)

        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"摄像头初始化后的FPS设置: {self.fps}")

        self.W = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.H = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.cur_frame_id = 0
        self.cam_type_state = cam_type_state
        self.stream_type = stream_type
        self.current_format = actual_format

        # 重构：使用单个高效的后台线程执行顺序读取与转码，彻底消除多线程 GIL 争抢与 CPU 忙等
        self.output_queue = queue.Queue(maxsize=max_queue_size)
        self.stop_event = threading.Event()

        self.read_thread = threading.Thread(target=self._frame_pipeline, daemon=True)
        self.read_thread.start()

    def _configure_camera_format(self, target_fourcc, width, height):
        fourcc = cv2.VideoWriter_fourcc(*target_fourcc)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 20)

    def _get_current_format(self):
        fourcc_code = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        return ''.join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])

    def _frame_pipeline(self):
        """高效单线程视频流处理管线"""
        consecutive_failures = 0
        while not self.stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                consecutive_failures += 1
                if consecutive_failures % 10 == 0:
                    print(f"[WARN] Camera {self.cam_type_state} read failed ({consecutive_failures} times consecutively)")
                if consecutive_failures > 50:
                    print(f"[ERROR] Camera {self.cam_type_state} read failed 50 times consecutively. Stopping.")
                    self.stop_event.set()
                    break
                self.stop_event.wait(0.05)
                continue
            consecutive_failures = 0

            # 校验数据形状
            if self.current_format == "MJPG":
                if frame.shape != (self.H, self.W, 3):
                    self.stop_event.set()
                    break

            # 顺序转码并推入输出队列（使用非阻塞 put_nowait 确保绝对实时性）
            yuv_bytes = bgr_to_nv12_bytes(frame)
            try:
                self.output_queue.put_nowait(yuv_bytes)
            except queue.Full:
                # 队列满时立刻丢弃最旧的一帧，并无延迟地压入最新帧，保证最高流畅度
                try:
                    _ = self.output_queue.get_nowait()
                    self.output_queue.put_nowait(yuv_bytes)
                except queue.Empty:
                    pass

    def read_frames(self):
        try:
            while not self.stop_event.is_set() or not self.output_queue.empty():
                try:
                    yuv_bytes = self.output_queue.get(timeout=0.5)
                    yield yuv_bytes
                    self.output_queue.task_done()
                except queue.Empty:
                    if self.stop_event.is_set():
                        break
        finally:
            self.stop_event.set()
            if self.read_thread.is_alive():
                self.read_thread.join()
            self.cap.release()

    def __del__(self):
        self.stop_event.set()
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
