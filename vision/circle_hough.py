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
        centroids, radii = [], []
        
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
            for (x, y, r) in circles[0]:
                if random_sampling_overlap(mask, (x, y), r, 100) > self.overlap:
                    centroids.append([x, y])
                    radii.append(r)

        if len(centroids):
            radii = np.array(radii)
            centroids = np.array(centroids)
            
            radii = np.mean(radii, axis=0)
            centroids = np.mean(centroids, axis=0)

            self.draw_circle(frame, centroids.astype(int), radii.astype(int))

        return frame, centroids, radii
    
    def draw_circle(self, frame, centroid, radius):
        cv.circle(frame, (int(centroid[0]), int(centroid[1])), int(radius), (0, 255, 0), 4)
        cv.circle(frame, (int(centroid[0]), int(centroid[1])), 2, (0, 0, 255), 3)
        return None
    
def random_sampling_overlap(mask, center, radius, num_samples=100):
    # Generate random angles and radii for the samples
    angles = np.random.uniform(0, 2 * np.pi, num_samples)
    radii = np.random.uniform(0, radius, num_samples) 
    
    # Convert polar coordinates to Cartesian coordinates
    sample_x = center[0] + (radii * np.cos(angles)).astype(int)
    sample_y = center[1] + (radii * np.sin(angles)).astype(int)
    
    # Ensure the sampled points are within the image bounds
    valid_x = np.clip(sample_x, 0, mask.shape[1] - 1).astype(int)
    valid_y = np.clip(sample_y, 0, mask.shape[0] - 1).astype(int)
    
    # Check if the sampled points overlap with the mask
    overlap_count = np.count_nonzero(mask[valid_y, valid_x] > 0)
    
    # Calculate the degree of overlap
    percentage_overlap = overlap_count / num_samples

    return percentage_overlap