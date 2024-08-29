import cv2 as cv
import numpy as np
from math import asin

import atexit

class Camera(object):
    def __init__(self, cam_idx=0):
        self.camera = cv.VideoCapture(cam_idx)
        if not self.camera.isOpened():
            raise Exception("Error: Could not open camera.")

        self.matrix = np.load('vision/params/camera_matrix.npy')
        self.dist_coeffs = np.load('vision/params/distortion.npy')

        atexit.register(self.release)
    
    def read_frame(self):
        success, frame = self.camera.read()
        if not success:
            raise Exception("Error: Could not read frame.")
        
        frame = cv.flip(frame, 0) # vertical
        frame =  cv.flip(frame, 1) # horizontal
        return frame
    
    def release(self):
        self.camera.release()
    
    def undistort(self, frame):
        return cv.undistort(frame, self.matrix, self.dist_coeffs)
    
    def distance(self, u, v, r_px, r_m):
        d = self.matrix[0, 0] * r_m / r_px
        y = (u - self.matrix[0, 2]) * (r_m / r_px)
        return d, asin(y / d)
