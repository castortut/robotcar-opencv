import cv2
import os
import time
import numpy as np
from PIL import Image
import glob
from matplotlib import pyplot as plt

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
        time.sleep(2)

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

i = 0

try:
    while True:
        frame = camera.run()
        #cv2.imshow('video', frame)
        #cv2.waitKey(1)

        f = f"/home/jetson/track_images/img{str(i).zfill(6)}.jpg"
        cv2.imwrite(f, frame)

        i += 1

        time.sleep(0.5)

finally:    
    camera.shutdown()
