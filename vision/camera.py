import cv2 as cv
import numpy as np
import atexit

class Camera(object):
    def __init__(self, cam_idx=0):
        self.camera = cv.VideoCapture(cam_idx)
        if not self.camera.isOpened():
            raise Exception("Error: Could not open camera.")

        self.matrix = np.load('vision/params/camera_matrix.npy')
        self.dist_coeffs = np.load('vision/params/distortion.npy')
        self.square_size = np.load('vision/params/square_size.npy')

        atexit.register(self.release)
    
    def read_frame(self):
        success, frame = self.camera.read()

        if not success:
            raise Exception("Error: Could not read frame.")
        
        return cv.flip(frame, 0) # vertical
    
    def release(self):
        self.camera.release()
    
    def undistort(self, frame):
        return cv.undistort(frame, self.matrix, self.dist_coeffs)
    
    def distance(self, u, v, r_px, r_m):
        y = self.square_size * self.matrix[0, 0] * r_m / r_px
        x = self.square_size * (u - self.matrix[0, 2]) * (y / self.matrix[0, 0])
        h = self.square_size * -(v - self.matrix[1, 2]) * (y / self.matrix[1, 1])
        return x, y, h
