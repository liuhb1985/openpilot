import av
import cv2
import threading
import queue
import os
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor


def bgr_to_nv12_bytes(bgr_frame):
    """Convert a BGR ndarray to NV12 bytes. Module-level so it can be used with ProcessPoolExecutor."""
    frame = av.VideoFrame.from_ndarray(bgr_frame, format='bgr24')
    return frame.reformat(format='nv12').to_ndarray().data.tobytes()

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
    def __init__(self, cam_type_state, stream_type, camera_id, num_workers=None, max_queue_size=10, use_processes=False):
        try:
            camera_id = int(camera_id)
        except ValueError:
            pass

        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise OSError(f"无法打开摄像头设备 {camera_id}")

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

        # 负责从摄像头读取帧的线程（主进程内读取）
        self.read_thread = threading.Thread(target=self._frame_reader, daemon=True)

        # 支持线程池或进程池用于帧转码（BGR -> NV12）
        self.use_processes = use_processes
        if self.use_processes:
            # 使用进程池时，确保转码函数可被 pickled（已在模块顶层）
            self.executor = ProcessPoolExecutor(max_workers=self.num_workers)
        else:
            self.executor = ThreadPoolExecutor(max_workers=self.num_workers)

        self.read_thread.start()

    def _configure_camera_format(self, target_fourcc):
        fourcc = cv2.VideoWriter_fourcc(*target_fourcc)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2592)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1944)
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
                # 将捕获的原始 BGR 帧放入队列，由 worker/进程池转码
                try:
                    self.frame_queue.put(frame, timeout=0.1)
                except queue.Full:
                    # 队列满时丢帧以避免阻塞摄像头读取
                    continue
            else:
                self.stop_event.wait(0.01)

    def _frame_worker(self):
        """Worker loop used when not using process pool: consume frames, convert and push to output queue."""
        while not self.stop_event.is_set() or not self.frame_queue.empty():
            try:
                frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if self.current_format == "MJPG":
                if frame.shape != (self.H, self.W, 3):
                    # 不要抛出异常阻塞整个管线，改为设置停止并记录
                    self.stop_event.set()
                    break
            yuv_bytes = bgr_to_nv12_bytes(frame)
            try:
                self.output_queue.put(yuv_bytes, timeout=0.1)
            except queue.Full:
                # 如果输出队列满，丢弃最旧的输出并放入新的
                try:
                    _ = self.output_queue.get_nowait()
                    self.output_queue.put(yuv_bytes)
                except queue.Empty:
                    pass
            finally:
                self.frame_queue.task_done()

    def _frame_worker_process_pool(self):
        """Submit frames to process pool for conversion and collect results."""
        futures = set()
        while not self.stop_event.is_set() or not self.frame_queue.empty() or futures:
            # submit new frames up to available worker slots
            try:
                while len(futures) < self.num_workers and not self.frame_queue.empty():
                    frame = self.frame_queue.get_nowait()
                    fut = self.executor.submit(bgr_to_nv12_bytes, frame)
                    futures.add(fut)
                    self.frame_queue.task_done()
            except queue.Empty:
                pass

            # collect completed futures
            done = [f for f in futures if f.done()]
            for f in done:
                futures.remove(f)
                try:
                    yuv_bytes = f.result()
                    try:
                        self.output_queue.put(yuv_bytes, timeout=0.1)
                    except queue.Full:
                        try:
                            _ = self.output_queue.get_nowait()
                            self.output_queue.put(yuv_bytes)
                        except queue.Empty:
                            pass
                except Exception:
                    # 忽略单帧转码错误，但标记停止以便调试
                    self.stop_event.set()
                    break

    def read_frames(self):
        workers = []
        # 启动合适的 worker：线程版或进程池版
        if self.use_processes:
            # 一个线程负责把队列中的帧提交到进程池并收集结果
            t = threading.Thread(target=self._frame_worker_process_pool)
            t.start()
            workers.append(t)
        else:
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
            # 关闭 executor
            try:
                self.executor.shutdown(wait=True)
            except Exception:
                pass
            self.cap.release()

    def __del__(self):
        self.stop_event.set()
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
