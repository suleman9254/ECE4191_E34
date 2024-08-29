from math import atan, atan2
from time import time, sleep
import numpy as np
from threading import Thread
import cv2 as cv

class Robot2(object):
    def __init__(self, camera, detector, model, controller, planner):
        self.camera = camera
        self.detector = detector
        self.model = model
        self.controller = controller
        self.planner = planner

    def drive(self, dt, goal_x=None, goal_y=None, goal_th=None, v=None, w=None):
        if v is None or w is None:
            v, w = self.planner.plan(goal_x, goal_y, goal_th, self.model.x, self.model.y, self.model.th)

        start_time = time()
        while time() - start_time < dt:
            duty_cycle_l, duty_cycle_r = self.controller.drive(v, w, self.model.wl, self.model.wr)
            self.model.pose_update(duty_cycle_l, duty_cycle_r)
    
    def brake(self):
        self.model.pose_update(duty_cycle_l=0, duty_cycle_r=0)

    def home(self):
        desired_heading = atan2(-self.model.y, -self.model.x)

        while True:
            self.drive(dt=0.01, goal_x=-0.1, goal_y=-0.1, goal_th=desired_heading)
            self.brake()

    def middle(self, dt, xbound, ybound):
        goal_x, goal_y = xbound / 2, ybound / 2
        goal_th = atan2(goal_y, goal_x)

        start_time = time()
        while time() - start_time < dt:
            self.drive(dt=0.01, goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)
    
    def find_ball(self, r_m):
        frame, dist = self.camera.read_frame(), None
        frame, centroid, r_px = self.detector.find_ball(frame)

        if len(centroid):
            dist, _, _ = self.camera.distance(*centroid, r_px, r_m)
        return frame, centroid, dist
    
    def orient(self, frame, centroid, center_tolerance, r_m, dt):
        print('Angle Correction!')
        self.model.th = 0
        while True:
            err = (frame.shape[0] / 2) - centroid[0]
            if abs(err) < center_tolerance:
                break

            v, w = (0, 0.5) if err < 0 else (0, -0.5)
            self.drive(dt=dt, v=v, w=w)
            self.brake()

            frame, centroid, _ = self.find_ball(r_m)
            if not centroid:
                return True  # lost the ball
        
        return False
    
    def detect_collect(self, r_m, max_dist, dt, center_tolerance, xbound, ybound):
        self.drive(dt=30, goal_x=xbound / 2, goal_y=ybound / 2, goal_th=atan2(ybound, xbound))

        while True:
            frame, centroid, dist = self.find_ball(r_m)
            
            if len(centroid):
                if abs(dist) > max_dist:     
                    lost = self.orient(frame, centroid, center_tolerance=center_tolerance, r_m=r_m, dt=0.1)
                    
                    if not lost:
                    
                        speed = max(0.1, min(1.0, 1.0 / dist))
                        self.drive(dt, self.model.x + dist, self.model.y, self.model.th, v=speed)
                        self.brake()
                
                else:
                    self.brake()  #
                    break
            
            else:
                goal_th = self.model.th - 1
                self.drive(dt=0.08, goal_x=self.model.x, goal_y=self.model.y, goal_th=goal_th)
                self.brake()
            
        
            if abs(self.model.x) < 0.5 and abs(self.model.y) < 0.5:
                speed = max(0.1, min(0.5, 0.5 / (abs(self.model.x) + abs(self.model.y))))
                self.drive(dt=dt, v=speed)
                self.brake()
