import cv2
import os
import time
import numpy as np
from PIL import Image
import glob


class BaseCamera:

    def run_threaded(self):
        return self.frame

'''
Camera module for Jetson Nano with RPiCam v2
'''
class CSICamera(BaseCamera):
    def gstreamer_pipeline(self,capture_width=240, capture_height=320, display_width=240, display_height=320, framerate=50, flip_method=0) :   
        return ('nvarguscamerasrc ! ' 
        'video/x-raw(memory:NVMM), '
        'width=(int)%d, height=(int)%d, '
        'format=(string)NV12, framerate=(fraction)%d/1 ! '
        'nvvidconv flip-method=%d ! '
        'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))
    
    def __init__(self, resolution=(120, 160), framerate=60):
        # initialize the camera and stream
        self.camera = cv2.VideoCapture(self.gstreamer_pipeline(display_width=resolution[1],display_height=resolution[0],flip_method=0), cv2.CAP_GSTREAMER)
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

try:
    while True:
        frame = camera.run()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv[:,:,2] = 128
        #hsv[:,:,1] = 128
        hsv[:,:,1] = 128*np.rint(hsv[:,:,1] / 128)
        hsv[:,:,0] = 128*np.rint(hsv[:,:,0] / 128)
        cv2.imshow('video', hsv)
        cv2.waitKey(1)

finally:    
    camera.shutdown()
