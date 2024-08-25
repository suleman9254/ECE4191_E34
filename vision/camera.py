import cv2 as cv
import numpy as np

class Camera(object):
    def __init__(self, cam_idx=0):
        self.camera = cv.VideoCapture(cam_idx)
        if not self.camera.isOpened():
            raise Exception("Error: Could not open camera.")

        self.matrix = np.array([[1.38133202e+03, 0.00000000e+00, 7.70244844e+02],
                                [0.00000000e+00, 1.38658277e+03, 3.70361811e+02],
                                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        
        self.dist_coeffs = np.array([[-2.69619382e-01, 
                                      -8.26450256e-01, 
                                      -1.65810414e-03, 
                                      -7.42041123e-03,
                                      6.46280851e+00]])
    
    def read_frame(self):
        success, frame = self.camera.read()

        if not success:
            raise Exception("Error: Could not read frame.")
        
        return frame
    
    def undistort(self, frame):
        return cv.undistort(frame, self.matrix, self.dist_coeffs)
    
    def distance(self, u, v, r_px, r_m):
        y = self.matrix[0, 0] * r_m / r_px
        x = (u - self.matrix[0, 2]) * (y / self.matrix[0, 0])
        h = -(v - self.matrix[1, 2]) * (y / self.matrix[1, 1])
        return x, y, h
