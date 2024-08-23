import cv2 as cv
import numpy as np

class Camera(object):
    def __init__(self, cam_idx):
        self.camera = cv.VideoCapture(cam_idx)
        if not self.camera.isOpened():
            raise Exception("Error: Could not open camera.")

        self.matrix = np.array([
            [649.47113436, 0.00000000, 249.59894886],
            [0.00000000, 650.00394065, 252.46785209],
            [0.00000000, 0.00000000, 1.00000000]
            ])
        
        self.dist_coeff = np.array([[-0.12860389, 
                                     -0.3649167, 
                                     0.00291756, 
                                     -0.00061939, 
                                     0.32511248]])
    
    def read_frame(self):
        success, frame = self.camera.read()

        if not success:
            raise Exception("Error: Could not read frame.")
        
        return frame
    
    def distance(self, u, v, r_px, r_cm):
        scale_a = r_cm / r_px
        scale_b = scale_a * (self.matrix[0, 0] / self.matrix[1, 1])

        x = scale_a * self.matrix[0, 0]
        y = scale_a * (u - self.matrix[0, 2])
        h = scale_b * (self.matrix[1, 2] - v)
        return x, y, h
