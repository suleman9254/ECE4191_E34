from math import atan
from time import time

class Robot(object):
    def __init__(self, camera, detector, model, controller, planner):

        self.camera = camera
        self.detector = detector
        self.model = model
        self.controller = controller
        self.planner = planner
    
    def find_ball(self, r_m):
        frame = self.camera.read_frame()
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
    
    def drive_to_pos(self, goal_x, goal_y, goal_th, tolerance):
        err_x = abs(self.model.x - goal_x)
        err_y = abs(self.model.y - goal_y)

        while err_x > tolerance or err_y > tolerance:
            self.drive_step(goal_x, goal_y, goal_th)

            err_x = abs(self.model.x - goal_x)
            err_y = abs(self.model.y - goal_y)
        
        return None

    def drive_for_time(self, goal_x, goal_y, goal_th, dur):
        start_time = time()
        while time() - start_time < dur:
            self.drive_step(goal_x, goal_y, goal_th)
        
        return None
    
