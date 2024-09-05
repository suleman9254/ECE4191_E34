from threading import Thread
from queue import Queue

import cv2 as cv
import numpy as np

import atexit
from math import asin

class Camera(object):
    def __init__(self, cam_idx=0):
        
        self.camera = cv.VideoCapture(cam_idx)
        if not self.camera.isOpened():
            raise Exception("Error: Could not open camera.")
        
        self.matrix = np.load('vision/calibration/params/camera_matrix.npy')
        self.dist_coeffs = np.load('vision/calibration/params/distortion.npy')
        
        self.queue = Queue()
        thread = Thread(target=self._reader)
        thread.daemon = True
        thread.start()
    
    def _reader(self):
        while True:
            success, frame = self.camera.read()
            if not success:
                raise Exception("Error: Could not read frame.")
            
            if not self.queue.empty():
                self.queue.get_nowait()

            self.queue.put(frame)
    
    def read_frame(self):
        frame = self.queue.get()
        frame = cv.flip(frame, 0) # vertical
        frame =  cv.flip(frame, 1) # horizontal
        return frame
    
    def undistort(self, frame):
        return cv.undistort(frame, self.matrix, self.dist_coeffs)
    
    def distance(self, u, v, r_px, r_m):
        d = self.matrix[0, 0] * r_m / r_px
        y = (u - self.matrix[0, 2]) * (r_m / r_px)
        return d, asin(y / d)
