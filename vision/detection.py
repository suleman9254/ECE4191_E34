from vision.circle_overlap import random_sampling_overlap
import numpy as np
import cv2 as cv

class Detector(object):
    def __init__(self, k=11, sig=3, hsv_low=[20, 0, 0], hsv_high=[50, 255, 255], dp=1.2, minDist=5, param1=100, param2=40, maxRadius=200, overlap=0.9):
        self.k, self.sig = k, sig
        self.dp, self.minDist = dp, minDist
        self.param1, self.param2 = param1, param2
        self.maxRadius, self.overlap = maxRadius, overlap
        
        self.hsv_low = np.array(hsv_low)
        self.hsv_high = np.array(hsv_high)
    
    def find_ball(self, frame):
        centroids = []
        
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray_blurred = cv.GaussianBlur(gray, (self.k, self.k), self.sig)

        circles = cv.HoughCircles(gray_blurred, 
                                  cv.HOUGH_GRADIENT,
                                  dp=self.dp,
                                  minDist=self.minDist,
                                  param1=self.param1,
                                  param2=self.param2,
                                  maxRadius=self.maxRadius)

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) # color masking
        mask = cv.inRange(hsv, self.hsv_low, self.hsv_high)

        if circles is not None:
            for (x, y, r) in circles:
                if random_sampling_overlap(mask, (x, y), r, 100) > self.overlap:
                    centroids.append([x, y])

        centroids = np.array(centroids)
        return np.mean(centroids) if len(centroids) else centroids