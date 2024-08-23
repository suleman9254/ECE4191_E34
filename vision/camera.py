import cv2 as cv
import numpy as np

class Camera(object):
    def __init__(self, cam_idx=0):
        self.camera = cv.VideoCapture(cam_idx)
        if not self.camera.isOpened():
            raise Exception("Error: Could not open camera.")

        self.matrix = np.array([[1.38133198e+03, 0.00000000e+00, 7.70244836e+02],
                                [0.00000000e+00, 1.38658278e+03, 3.70361827e+02],
                                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        
        self.dist_coeffs = np.array([[-2.69618894e-01, 
                                      -8.26454741e-01, 
                                      -1.65811855e-03, 
                                      -7.42042402e-03, 
                                      6.46282805e+00]])
    
    def read_frame(self):
        success, frame = self.camera.read()

        if not success:
            raise Exception("Error: Could not read frame.")
        
        return frame
    
    def distance(self, u, v, r_px, r_cm):
        z = self.matrix[0, 0] * r_cm / r_px
        x = (u - self.matrix[0, 2]) * (z / self.matrix[0, 0])
        y = (v - self.matrix[1, 2]) * (z / self.matrix[1, 1])
        return x, y, z
