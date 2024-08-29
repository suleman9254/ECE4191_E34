from math import atan
from time import time, sleep
import numpy as np
from threading import Thread
import cv2 as cv
import math

class Robot3(object):
    def __init__(self, camera, detector, model, controller, planner):

        self.camera = camera
        self.detector = detector
        self.model = model
        self.controller = controller
        self.planner = planner

    def drive_step(self, goal_x, goal_y, goal_th, rotate):
        v, w = self.planner.plan(goal_x, goal_y, goal_th, self.model.x, self.model.y, self.model.th,)
        duty_cycle_l, duty_cycle_r = self.controller.drive(v, w, self.model.wl, self.model.wr, rotate)
        self.model.pose_update(duty_cycle_l, duty_cycle_r)
        self.model.pose_update(duty_cycle_l=0, duty_cycle_r=0)
        return None
    
    def find_ball(self, r_m):
        frame = self.camera.read_frame()
        frame, centroid, r_px = self.detector.find_ball(frame)

        if len(centroid):
            dist, _, _ = self.camera.distance(*centroid, r_px, r_m)
        else:
            dist = None

        return frame, centroid, dist
    
    def angle_correct(self, frame, centroid, cntr_px_tol, r_m,rotate):
        err = (frame.shape[0] / 2) - centroid[0]
        while abs(err) > cntr_px_tol:

            v, w = (0, 0.5) if err < 0 else (0, -0.5)
            duty_cycle_l, duty_cycle_r = self.controller.drive(v, w, self.model.wl, self.model.wr, False)
            self.model.pose_update(duty_cycle_l, duty_cycle_r)

            self.model.pose_update(duty_cycle_l=0, duty_cycle_r=0)

            frame, centroid, _ = self.find_ball(r_m)
            if not len(centroid):
                return True
            
            err = (frame.shape[0] / 2) - centroid[0]

        return 
    
    def return_home(self):            
        desired_heading = math.atan2(-self.model.y, -self.model.x)
        relative_heading = desired_heading - self.model.th
        relative_heading = (relative_heading + math.pi) % (2 * math.pi) - math.pi

        while True:
            self.drive_step(0, 0, desired_heading , True)
            print(self.model.x)
            print(self.model.y)
    
        return None

    def detect_collect(self, r_m, max_dist, cntr_px_tol):
        # self.return_home()
        while True:
            frame, centroid, dist = self.find_ball(r_m)
            cv.imwrite(f'tests/{time()}.jpg', frame)          
            
            if len(centroid):
                if abs(dist) > max_dist:     
                    
                    print('Dist!')
                    print(abs(dist))     

                    lost = self.angle_correct(frame, centroid, cntr_px_tol, r_m, True)
                    # print(self.model.x)
                    # print(self.model.y)
                    # print(self.model.th)
                    # print('----')

                    if not lost:
                        start_time = time()
                        while time() - start_time < 2:
                            # print('Entering Forward Drive')
                            # print(self.model.x)
                            # print(self.model.y)
                            # print(self.model.th)
                            # print('----')
                            self.drive_step(self.model.x + dist, self.model.y, self.model.th, 2)

                    # print('Finished Driving Forward')
                else:
                    self.return_home()
