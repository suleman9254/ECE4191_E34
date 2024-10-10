from vision.camera import Camera
from vision.detector import YOLODetector

import cv2 as cv

box_ckpt, ball_ckpt = r'vision/ckpts/boxes_new.pt', r'vision/ckpts/balls_old.pt'
camera, detector = Camera(0), YOLODetector(box_ckpt=box_ckpt, ball_ckpt=ball_ckpt, box_thresh=0.1, ball_thresh=0.1)

frame = camera.read_frame()
print(frame.shape)
frame = camera.undistort(frame)

frame, centroid, pixels = detector.find_ball(frame)

print(pixels)

dist, th = camera.distance(*centroid, pixels, r_m=0.0335)

print(dist)

detector.draw_circle(frame, centroid, pixels)

cv.imwrite('test3.jpg', frame)

