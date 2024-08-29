import cv2 as cv
import numpy as np
import atexit

class Camera(object):
    def __init__(self, cam_idx=0):
        self.camera = cv.VideoCapture(cam_idx)
        if not self.camera.isOpened():
            raise Exception("Error: Could not open camera.")
        print("Camera successfully opened.")

        try:
            self.matrix = np.load('vision/params/camera_matrix.npy')
            self.dist_coeffs = np.load('vision/params/distortion.npy')
            print("Camera calibration data loaded successfully.")
        except FileNotFoundError:
            raise Exception("Error: Calibration data files not found.")

        atexit.register(self.release)
    
    def read_frame(self):
        success, frame = self.camera.read()
        print('hello')  # Debugging print statement
        if not success:
            raise Exception("Error: Could not read frame.")
        
        frame = cv.flip(frame, 0)  # Vertical flip
        frame = cv.flip(frame, 1)  # Horizontal flip
        return frame
    
    def release(self):
        print("Releasing camera resources.")
        self.camera.release()
    
    def undistort(self, frame):
        if self.matrix is None or self.dist_coeffs is None:
            raise Exception("Error: Camera calibration data is missing.")
        print("Undistorting frame.")
        return cv.undistort(frame, self.matrix, self.dist_coeffs)
    
    def distance(self, u, v, r_px, r_m):
        """
        Calculate the real-world coordinates (x, h, y) of the detected object.
        
        Parameters:
        - u, v: The pixel coordinates of the object in the image.
        - r_px: The radius of the object in pixels.
        - r_m: The real-world radius of the object in meters.
        
        Returns:
        - (x, h, y): The real-world coordinates of the object.
        """
        x = self.matrix[0, 0] * r_m / r_px
        y = (u - self.matrix[0, 2]) * (x / self.matrix[0, 0])
        h = -(v - self.matrix[1, 2]) * (x / self.matrix[1, 1])
        print(f"Calculated distance: x={x}, h={h}, y={y}")
        return x, h, y
