import numpy as np
from time import time
from math import cos, sin, atan2, pi

class Robot(object):
    def __init__(self, model, controller, planner, claw=None, rail=None, sensor=None, camera=None, detector=None, params=None):

        self.claw = claw
        self.camera = camera
        self.detector = detector
        self.model = model
        self.controller = controller
        self.planner = planner
        self.sensor = sensor
        self.rail = rail

        if params is not None:
            self.xBox = params['xBox']
            self.yBox = params['yBox']
            self.xBound = params['xBound']
            self.yBound = params['yBound']
            self.ballRadius = params['ballRadius']
            self.boxHeight = params['boxHeight']

        self.timeout = 1
        self.explore_state = 0

    def home(self):
        x, y, _ = self.model.position()
        return 0, 0, atan2(y, x)
    
    def drive(self, goal_x, goal_y, goal_th):
        
        time_of_movement = time()
        goal_v = goal_w = float('inf')
        
        wl, wr, v, w = self.model.velocities()
        x, y, th = x0, y0, th0 = self.model.position()
        
        while not v == goal_v == w == goal_w == 0:

            goal_v, goal_w = self.planner.plan(goal_x, goal_y, goal_th, x, y, th)
            dutyL, dutyR = self.controller.drive(goal_v, goal_w, wl, wr)
            self.model.pose_update(dutyL, dutyR)

            x, y, th = self.model.position()
            wl, wr, v, w = self.model.velocities()
            
            if v != 0 or w != 0:
                time_of_movement = time()

            if time() - time_of_movement > self.timeout:
                break
        
        self.model.brake()

        if (x, y, th) == (x0, y0, th0):
            return False
        
        return True
        
    def transform(self, dist, th, alpha=1, beta=1):
        
        dist, th = alpha * dist, beta * th
        local_x, local_y = dist * cos(th), dist * sin(th)

        global_x = self.model.x + local_x * cos(self.model.th) - local_y * sin(self.model.th)
        global_y = self.model.y + local_x * sin(self.model.th) + local_y * cos(self.model.th)
        
        global_th = self.model.th + th
        return global_x, global_y, global_th

    def vision(self, length, detector):
        
        dist, th = None, None
        global_x, global_y = None, None

        frame = self.camera.read_frame()
        frame = self.camera.undistort(frame)
        frame, centroid, pixels = detector(frame)
        
        if centroid is not None:
            dist, th = self.camera.distance(*centroid, pixels, length)
            global_x, global_y, _ = self.transform(dist, th)

        return dist, th, global_x, global_y
    
    def simple_collect(self, distance):

        distance = distance - 0.02
        goal_x, goal_y, goal_th = self.transform(distance, th=0)
        _ = self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)        
        
        self.claw.grab()
        self.rail.set_position(0.3)
        return True

    def sensor_collect(self, grab_dist, alpha):
        
        movement = True
        while self.sensor.read() > grab_dist and movement:
            goal_x, goal_y, goal_th = self.transform(self.sensor.read(), th=0, alpha=alpha)
            movement = self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)

        self.claw.grab()

        if self.sensor.read() > grab_dist * 1.1:
            self.claw.release()
            return False

        self.rail.set_position(0.3)
        
        return True
    
    def approach(self, size, xbound, ybound, tgtProx, slwProx, alpha, beta, detector):
        
        movement = True
        goal_x = goal_y = goal_th = None

        while movement:
            dist, th, glob_x, glob_y = self.vision(size, detector)
            
            # undo last movement if ball is lost
            if dist is None and not (goal_x == goal_y == goal_th == None):
                self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)
                goal_x = goal_y = goal_th = None
                continue
            
            # out of bounds or no ball found
            if dist is None or glob_x > xbound or -glob_y > ybound:
                return False, None

            # last minute angle correction 
            dist_hat = 0 if dist < tgtProx else dist

            # smaller movements at close proximity
            alpha_hat = alpha / 2 if dist < slwProx else alpha

            goal_x, goal_y, goal_th = self.transform(dist_hat, th, alpha_hat, beta)
            movement = self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)

        return True, dist
    
    def deliver(self, tgtProx, slwProx, alpha, beta):

        x, y, _ = self.model.position()
        th = atan2(self.yBox, self.xBox)
        self.drive(goal_x=x, goal_y=y, goal_th=th)
        
        xbound = ybound = float('inf')
        detector = self.detector.find_box
        detected, dist = self.approach(self.boxHeight, xbound, ybound, tgtProx, slwProx, alpha, beta, detector)

        if not detected:
            return False

        self.rail.set_position(1.0)
        x, y, th = self.transform(dist - 0.002, th=0)
        self.drive(goal_x=x, goal_y=y, goal_th=th)
        
        self.claw.release()  

        x, y, th = x - 0.4, y - 0.4, -th
        self.drive(goal_x=x, goal_y=y, goal_th=th)
        self.rail.set_position(0)

        return True
    
    def explore(self):
        
        x, y, th = self.model.position()
        th = th + 2*pi / 8 if self.explore_state < 8 else th + 2*pi / 16

        self.drive(goal_x=x, goal_y=y, goal_th=th)
        self.explore_state = self.explore_state + 1
    
    def start(self, onTime, maxCamBallDist, maxCamBoxDist, grabDist, alphaReductionDist, alpha, beta):
        
        start_time = time()
        while time() - start_time < onTime:
            
            detector = self.detector.find_ball
            detected, dist = self.approach(self.ballRadius, self.xBound, self.yBound, maxCamBallDist, alphaReductionDist, alpha, beta, detector)

            if not detected:
                self.explore(); continue
            
            self.explore_state = 0
            collected = self.simple_collect(dist)
            
            if not collected:
                continue
            
            delivered = self.deliver(maxCamBoxDist, alphaReductionDist, alpha, beta)
            while not delivered:
                self.explore(); delivered = self.deliver(maxCamBoxDist, alphaReductionDist, alpha, beta)
            
            self.explore_state = 0

        home = self.home()
        self.drive(*home)