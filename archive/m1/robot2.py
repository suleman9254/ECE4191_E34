from gpiozero import LED

from math import atan
from time import time, sleep
import numpy as np
from threading import Thread
import cv2 as cv
from math import atan, atan2

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
            self.drive(dt=0.01, goal_x=-0.3, goal_y=-0.3, goal_th=desired_heading)

    def middle(self, dt, xbound, ybound):
        self.drive(dt=dt, goal_x=xbound/2, goal_y=0, goal_th=0)
        # start_time = time()
        # while time() - start_time < dt:
            # self.drive(dt=0.01, goal_x=xbound/2, goal_y=0, goal_th=0)
            # self.drive(dt=0.01, v=100, w=0)

            # if self.model.x > xbound:
                # break
    
    def find_ball(self, r_m):
        frame, dist = self.camera.read_frame(), None
        frame, centroid, r_px = self.detector.find_ball(frame)

        if len(centroid):
            dist, _, _ = self.camera.distance(*centroid, r_px, r_m)
        return frame, centroid, dist
    
    def orient(self, frame, centroid, thresh, r_m, dt):
        print('Angle Correction!')
        # self.model.th = 0
        while True:
            
            err = (frame.shape[0] / 2) - centroid[0]
            if abs(err) < thresh:
                break

            v, w = (0, 0.5) if err < 0 else (0, -0.5)
            self.drive(dt=dt, v=v, w=w)
            self.brake()

            frame, centroid, _ = self.find_ball(r_m)
            if not centroid:
                return True # lost the ball
        
        return False
    
    def detect_collect(self, r_m, max_dist, dt, center_tolerance, xbound, ybound):
        self.middle(dt=4, xbound=xbound, ybound=ybound)

        while True:
            frame, centroid, dist = self.find_ball(r_m)
            
            if len(centroid):
                if abs(dist) > max_dist:     
                    
                    lost = self.orient(frame, centroid, thresh=center_tolerance, r_m=r_m, dt=0.1)

                    if not lost:
                        new_dt = 0.1 if abs(dist) < 0.5 else dt
                        
                        self.drive(new_dt, self.model.x + dist, self.model.y, self.model.th)
                        self.brake()
                
                else:
                    led = LED(4)
                    led.on()
                    self.home()
            
            else:
                goal_th = self.model.th - 1
                self.drive(dt=0.08*2, goal_x=self.model.x, goal_y=self.model.y, goal_th=goal_th)
                self.brake()