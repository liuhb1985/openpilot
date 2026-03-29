import cv2

common_resolutions = [
    (1920, 1080),
    (1280, 720),
    (1024, 768),
    (800, 600),
    (640, 480),
    (320, 240)
]

for i in range(20):
    # Ubuntu 下使用 V4L2 或默认即可
    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)  # 或直接 cap = cv2.VideoCapture(i)
    if cap.isOpened():
        # 获取当前分辨率
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"摄像头索引 {i} 可用, 当前分辨率: {width}x{height}")

        # 尝试常见分辨率
        supported = []
        for w, h in common_resolutions:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if actual_w == w and actual_h == h:
                supported.append(f"{w}x{h}")
        print(f"摄像头索引 {i} 支持的分辨率（尝试常见值）: {supported}")

        cap.release()
    else:
        print(f"摄像头索引 {i} 打不开")
