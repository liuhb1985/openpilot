import av
import cv2
import threading
import queue
import os
from concurrent.futures import ThreadPoolExecutor

class Camera:
    def __init__(self, cam_type_state, stream_type, camera_id):
        try:
            camera_id = int(camera_id)
        except ValueError: # allow strings, ex: /dev/video0
            pass
        self.cam_type_state = cam_type_state
        self.stream_type = stream_type
        self.cur_frame_id = 0

        self.container = av.open(camera_id)
        assert self.container.streams.video, f"Can't open video stream for camera {camera_id}"
        self.video_stream = self.container.streams.video[0]
        self.W = self.video_stream.codec_context.width
        self.H = self.video_stream.codec_context.height

    @classmethod
    def bgr2nv12(self, bgr):
        frame = av.VideoFrame.from_ndarray(bgr, format='bgr24')
        return frame.reformat(format='nv12').to_ndarray()

    def read_frames(self):
        for frame in self.container.decode(self.video_stream):
            img = frame.to_rgb().to_ndarray()[:,:, ::-1] # convert to bgr24
            yuv = Camera.bgr2nv12(img)
            yield yuv.data.tobytes()
        self.container.close()

class CameraMJPG:
    def __init__(self, cam_type_state, stream_type, camera_id, num_workers=None, max_queue_size=10):
        try:
            camera_id = int(camera_id)
        except ValueError:
            pass

        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise IOError(f"无法打开摄像头设备 {camera_id}")

        self._configure_camera_format("MJPG")
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

        # 多线程相关
        if num_workers is None:
            num_workers = os.cpu_count() or 4
        self.num_workers = num_workers
        self.frame_queue = queue.Queue(maxsize=max_queue_size)
        self.output_queue = queue.Queue(maxsize=max_queue_size)
        self.stop_event = threading.Event()
        self.read_thread = threading.Thread(target=self._frame_reader)
        self.executor = ThreadPoolExecutor(max_workers=self.num_workers)

        self.read_thread.start()

    def _configure_camera_format(self, target_fourcc):
        fourcc = cv2.VideoWriter_fourcc(*target_fourcc)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 20)

    def _get_current_format(self):
        fourcc_code = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        return ''.join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])

    @staticmethod
    def _bgr_to_nv12(bgr_frame):
        frame = av.VideoFrame.from_ndarray(bgr_frame, format='bgr24')
        return frame.reformat(format='nv12').to_ndarray().data.tobytes()

    def _frame_reader(self):
        while not self.stop_event.is_set():
            if not self.frame_queue.full():
                ret, frame = self.cap.read()
                if not ret:
                    self.stop_event.set()
                    break
                self.frame_queue.put(frame)
            else:
                self.stop_event.wait(0.01)

    def _frame_worker(self):
        while not self.stop_event.is_set() or not self.frame_queue.empty():
            try:
                frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if self.current_format == "MJPG":
                if frame.shape != (self.H, self.W, 3):
                    raise ValueError("MJPG 解码后帧形状异常，请检查摄像头设置")
                yuv_bytes = self._bgr_to_nv12(frame)
            else:
                yuv_bytes = self._bgr_to_nv12(frame)
            self.output_queue.put(yuv_bytes)
            self.frame_queue.task_done()

    def read_frames(self):
        workers = []
        for _ in range(self.num_workers):
            t = threading.Thread(target=self._frame_worker)
            t.start()
            workers.append(t)

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
            self.read_thread.join()
            for t in workers:
                t.join()
            self.cap.release()

    def __del__(self):
        self.stop_event.set()
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
