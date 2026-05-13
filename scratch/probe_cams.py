import cv2
import os

def probe_cameras():
    print("Probing video devices in /dev/...")
    video_devices = sorted([f for f in os.listdir('/dev') if f.startswith('video')])
    
    for dev in video_devices:
        path = f"/dev/{dev}"
        cap = cv2.VideoCapture(path)
        if cap.isOpened():
            w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = cap.get(cv2.CAP_PROP_FPS)
            backend = cap.getBackendName()
            print(f"Device {path}: {w}x{h} @ {fps} fps (Backend: {backend})")
            
            # Try to grab one frame to ensure it's working
            ret, frame = cap.read()
            if ret:
                print(f"  Successfully captured frame from {path}")
            else:
                print(f"  Failed to capture frame from {path}")
            cap.release()
        else:
            print(f"Device {path}: Could not open")

if __name__ == "__main__":
    probe_cameras()
