from math import atan
from time import time, sleep
import numpy as np
from threading import Thread

class Robot(object):
    def __init__(self, camera, detector, model, controller, planner):

        self.camera = camera
        self.detector = detector
        self.model = model
        self.controller = controller
        self.planner = planner
    
    def find_ball(self, r_m):
        frame = self.camera.read_frame()
        frame = self.camera.undistort(frame) # undistort before detect

        frame, centroid, r_px = self.detector.find_ball(frame)
        goal_x, goal_y, goal_th = None, None, None

        if len(centroid):
            dx, dy, _ = self.camera.distance(*centroid, r_px, r_m)
            goal_x, goal_y = self.model.x + dx, self.model.y + dy
            goal_th = atan(dx / dy) + self.model.th
        
        return frame, goal_x, goal_y, goal_th
    
    def drive_step(self, goal_x, goal_y, goal_th):
        v, w = self.planner.plan(goal_x, goal_y, goal_th, self.model.x, self.model.y, self.model.th)
        duty_cycle_l, duty_cycle_r = self.controller.drive(v, w, self.model.wl, self.model.wr)
        self.model.pose_update(duty_cycle_l, duty_cycle_r)
        return None

    def detect_collect(self, r_m, bounds_x, bounds_y, max_dist, drive_duration, home_tolerance):
        while True:
            _, goal_x, goal_y, goal_th = self.find_ball(r_m)

            if goal_x is not None:

                if abs(goal_x) < bounds_x and \
                        abs(goal_y) < bounds_y:
                        
                        if abs(goal_x - self.model.x) > max_dist:
                        
                            start_time = time()
                            while time() - start_time < drive_duration:
                                self.drive_step(goal_x, goal_y, goal_th)
                            
                            self.model.pose_update(duty_cycle_l=0, duty_cycle_r=0)
                        
                        else:
                            
                            while abs(self.model.x) > home_tolerance or \
                                    abs(self.model.y) > home_tolerance:
                                self.drive_step(goal_x=0, goal_y=0, goal_th=0)

                            self.model.pose_update(duty_cycle_l=0, duty_cycle_r=0)
                            
                            return None