import cv2 as cv

class Camera(object):
    def __init__(self, cam_idx):
        self.camera = cv.VideoCapture(cam_idx)
        
        if not self.camera.isOpened():
            raise Exception("Error: Could not open camera.")
    
    def read_frame(self):
        success, frame = self.camera.read()

        if not success:
            raise Exception("Error: Could not read frame.")
        
        return frame