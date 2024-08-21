import numpy as np
import cv2 as cv

class Detector(object):
    def __init__(self, cfg):
        self.cfg = cfg
    
    def find_ball(self, frame):
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) # color masking
        mask = cv.inRange(hsv, self.cfg.hsv_low, self.cfg.hsv_high)
        masked_frame = cv.bitwise_and(frame, frame, mask=mask)

        cv.imwrite('test.jpg', masked_frame)
        
        gray = cv.cvtColor(masked_frame, cv.COLOR_BGR2GRAY)
        gray_blurred = cv.GaussianBlur(gray, (self.cfg.gauss_k, self.cfg.gauss_k), self.cfg.gauss_sig)

        circles = cv.HoughCircles(gray_blurred, 
                                  cv.HOUGH_GRADIENT,
                                  dp=self.cfg.cht_dp,
                                  minDist=self.cfg.cht_min_d,
                                  param1=self.cfg.cht_p1,
                                  param2=self.cfg.cht_p2,
                                  minRadius=self.cfg.cht_min_r,
                                  maxRadius=self.cfg.cht_max_r)

        # if circles is not None:
        #     circles = np.amax(circles[0], axis=0)

        return circles[0] if circles is not None else circles

class DetectorConfig(object):
    def __init__(self, 
                 cht_dp=1.2, 
                 cht_min_d=30, 
                 cht_p1=50, 
                 cht_p2=30, 
                 cht_min_r=10, 
                 cht_max_r=100, 
                 gauss_k=9, 
                 gauss_sig=2, 
                 hsv_low=[0, 0, 0], 
                 hsv_high=[255, 255, 255]):
    
        self.cht_dp = cht_dp
        self.cht_min_d = cht_min_d
        self.cht_p1 = cht_p1
        self.cht_p2 = cht_p2
        self.cht_min_r = cht_min_r
        self.cht_max_r = cht_max_r
        
        self.gauss_k = gauss_k
        self.gauss_sig = gauss_sig
        
        self.hsv_low = np.array(hsv_low)
        self.hsv_high = np.array(hsv_high)