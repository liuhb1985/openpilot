import cv2 as cv
import time

video = cv.VideoCapture()     # 调用摄像头，PC电脑中0为内置摄像头，1为外接摄像头
#video = cv2.VideoCapture(2)     # 调用摄像头，PC电脑中0为内置摄像头，1为外接摄像头
video.open(2, apiPreference=cv.CAP_V4L2)#

print("focus = ", video.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G')))
video.set(cv.CAP_PROP_FRAME_WIDTH, 1920)#1920
video.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)#1080
video.set(cv.CAP_PROP_FPS, 30)
video.set(cv.CAP_PROP_BRIGHTNESS,-10)
#video.set(cv.CAP_PROP_AUTO_EXPOSURE, 3)
#video.set(cv.CAP_PROP_EXPOSURE, 50)


print("w = ", video.get(cv.CAP_PROP_FRAME_WIDTH))
print("h = ", video.get(cv.CAP_PROP_FRAME_HEIGHT))
print("fps = ", video.get(cv.CAP_PROP_FPS))



start = int(round(time.time() * 1000))

out = cv.VideoWriter('output.mp4', cv.VideoWriter_fourcc(*'mp4v'), 20.0, (1920, 1080))
#out = cv.VideoWriter('output.mp4', cv.VideoWriter_fourcc('M','P','4','2'), 20.0, (1920, 1080))

#cv2.VideoWriter_fourcc(*'X265')

count = 0
judge = video.isOpened()      # 判断video是否打开

while judge:
    count = count + 1

    b = int(round(time.time() * 1000))
    ret, frame = video.read()
    out.write(frame)
    print("frame = ",int(round(time.time() * 1000)) - b)

    end = int(round(time.time() * 1000))
    if end - start > 1000:
        print("fps = ", count)
        count = 0
        start = end
        cv.imwrite("testw.jpg", frame)

    #if count%20 == 0:
        #end = int(round(time.time() * 1000))
        #print(" cost = ", end - start)
        #start = end


# 释放窗口
out.release()
video.release()
