import cv2
import os
import time
import numpy as np
from PIL import Image
import glob
from matplotlib import pyplot as plt
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685(busnum=1)
pwm.set_pwm_freq(60)
servo_min = 310
servo_max = 430

def set_angle(angle):
    angle = (-1.5*angle + 1)  / 2
    print(angle)
    val = int(servo_min + (servo_max - servo_min) * angle)
    pwm.set_pwm(0, 0, val)

WIDTH = 640
HEIGHT = 480
FPS = 50

class BaseCamera:

    def run_threaded(self):
        return self.frame

'''
Camera module for Jetson Nano with RPiCam v2
'''
class CSICamera(BaseCamera):
    def gstreamer_pipeline(self,capture_width=WIDTH, capture_height=HEIGHT, display_width=WIDTH, display_height=HEIGHT, framerate=FPS, flip_method=0):
        return ('nvarguscamerasrc ! ' 
        'video/x-raw(memory:NVMM), '
        'width=(int)%d, height=(int)%d, '
        'format=(string)NV12, framerate=(fraction)%d/1 ! '
        'nvvidconv flip-method=%d ! '
        'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))
    
    def __init__(self, resolution=(WIDTH, HEIGHT), framerate=FPS):
        # initialize the camera and stream
        self.camera = cv2.VideoCapture(self.gstreamer_pipeline(display_width=resolution[0],display_height=resolution[1],flip_method=6), cv2.CAP_GSTREAMER)
        self.ret , self.image = self.camera.read()
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None

        print('CSICamera loaded.. .warming camera')
        #time.sleep(2)

    def run(self):
        ret, frame = self.camera.read()
        return frame
    
    def shutdown(self):
        # indicate that the thread should be stopped
        print('stoping CSICamera')
        time.sleep(.5)
        del(self.camera)

camera = CSICamera()

COLOR_RANGE_BEGIN = 20
COLOR_RANGE_END = 50
COLOR_BRACKET = 10

fig = plt.figure()
ax = fig.add_subplot(121)

frame = camera.run()
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

hist = cv2.calcHist([hsv],[0],None,[256],[0,256])
line, = ax.plot(hist)

#plt.ion()
#plt.show()

try:
    while True:
        frame = camera.run()
        blur = cv2.GaussianBlur(frame, (15,15), 2)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        #hsv[:,:,2] = 128
        #hsv[:,:,1] = 128
        #hsv[:,:,1] = 128*np.rint(hsv[:,:,1] / 128)
        #hsv[:,:,0] = 128*np.rint(hsv[:,:,0] / 128)


        #frame = cv2.cvtColor(threshold, cv2.COLOR_HSV2BGR)

        hist = cv2.calcHist([hsv],[0],None,[256],[0,256])
        #line.set_ydata(hist)
        #fig.canvas.draw()

        hue_peak = np.argmax(hist[COLOR_RANGE_BEGIN:COLOR_RANGE_END]) + COLOR_RANGE_BEGIN
        #print(hue_peak)

        #if hue_peak < 6:
        #hue_peak = 15
        
        threshold = cv2.inRange(hsv, (int(hue_peak - COLOR_BRACKET), 50, 50), (int(hue_peak + COLOR_BRACKET), 255, 255))
        #imgt = cv2.bitwise_and(frame, frame, mask=threshold)

        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.filterByCircularity = False
        params.filterByConvexity = True
        params.minConvexity = 0.85
        params.filterByInertia = False
        params.minInertiaRatio = 0.2
        params.filterByColor = False
        params.minArea = 800
        params.maxArea = 999999

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(threshold)

        thr_with_keypoints = cv2.drawKeypoints(threshold, keypoints, np.array([]), (0,0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        #cv2.imshow('video', blur)
        #cv2.imshow('video', thr_with_keypoints)
        #cv2.waitKey(1)

        min_y = 9999
        x_coord = None
        for point in keypoints:
            if point.pt[1] < min_y:
                min_y = point.pt[1]
                x_coord = point.pt[0]
        print(x_coord)
        if x_coord:
            angle = (x_coord-(WIDTH/2)) / (WIDTH/2)
            set_angle(angle)


finally:    
    camera.shutdown()
