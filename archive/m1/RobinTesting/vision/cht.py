# from vision.circle_overlap import random_sampling_overlap
# import numpy as np
# import cv2 as cv

# class Detector(object):
#     def __init__(self, k=11, sig=3, hsv_low=[20, 0, 0], hsv_high=[50, 255, 255], dp=1.2, minDist=5, param1=100, param2=40, maxRadius=200, overlap=0.9):
#         self.k, self.sig = k, sig
#         self.dp, self.minDist = dp, minDist
#         self.param1, self.param2 = param1, param2
#         self.maxRadius, self.overlap = maxRadius, overlap
        
#         self.hsv_low = np.array(hsv_low)
#         self.hsv_high = np.array(hsv_high)
    
#     def find_ball(self, frame):
#         """
#         Detects balls in a given frame using Hough Circle Transform and HSV masking.
        
#         Parameters:
#         - frame: The input image frame (BGR format).
        
#         Returns:
#         - frame: The frame with detected circles drawn.
#         - centroids: The centroids of detected balls.
#         - radii: The radii of detected balls.
#         """
#         centroids, radii = [], []
        
#         # Convert frame to grayscale for circle detection
#         gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#         gray_blurred = cv.GaussianBlur(gray, (self.k, self.k), self.sig)

#         # Perform Hough Circle Transform to detect circles
#         circles = cv.HoughCircles(gray_blurred, 
#                                   cv.HOUGH_GRADIENT,
#                                   dp=self.dp,
#                                   minDist=self.minDist,
#                                   param1=self.param1,
#                                   param2=self.param2,
#                                   maxRadius=self.maxRadius)

#         # Convert frame to HSV for color masking
#         hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
#         mask = cv.inRange(hsv, self.hsv_low, self.hsv_high)

#         if circles is not None:
#             for (x, y, r) in circles[0]:
#                 # Ensure circles overlap significantly with the color mask
#                 if random_sampling_overlap(mask, (x, y), r, 100) > self.overlap:
#                     centroids.append([x, y])
#                     radii.append(r)

#         if centroids:
#             radii = np.array(radii)
#             centroids = np.array(centroids)
            
#             # Compute mean centroid and radius for detected circles
#             mean_radius = np.mean(radii, axis=0)
#             mean_centroid = np.mean(centroids, axis=0)

#             self.draw_circle(frame, mean_centroid.astype(int), int(mean_radius))

#         else:
#             # Handle the case where no circles are detected
#             print("No circles detected.")

#         return frame, centroids, radii
    
#     def draw_circle(self, frame, centroid, radius):
#         """
#         Draws a circle and its centroid on the frame.
        
#         Parameters:
#         - frame: The input image frame.
#         - centroid: The centroid of the circle.
#         - radius: The radius of the circle.
#         """
#         cv.circle(frame, (centroid[0], centroid[1]), radius, (0, 255, 0), 4)
#         cv.circle(frame, (centroid[0], centroid[1]), 2, (0, 0, 255), 3)
