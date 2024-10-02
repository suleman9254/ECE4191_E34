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

        self.exploreState = 0
        self.driveTimeout = 1
        self.collectTimeout = 30

    def home(self):
        x, y, _ = self.model.position()
        return 0, 0, atan2(y, x)
    
    def drive(self, goal_x, goal_y, goal_th):
        
        moveTime = time()
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
                moveTime = time()

            if time() - moveTime > self.driveTimeout:
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

    def collect(self, dist, grabDist, alpha):
        
        goal_x, goal_y, goal_th = self.transform(dist - 0.0025, th=0)
        _ = self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)

        start_time = time()
        while (currentDist := self.sensor.read()) > grabDist:

            goal_x, goal_y, goal_th = self.transform(currentDist, th=0, alpha=alpha)
            movement = self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)

            lost = self.sensor.read() > currentDist * 1.1
            timeout = time() - start_time > self.collectTimeout
            
            if not movement or lost or timeout:
                return False
        
        self.claw.grab()
        if self.sensor.read() > grabDist * 1.1:
            self.claw.release(); return False
        
        self.rail.set_position(0.1)
        if self.sensor.read() > grabDist * 1.1:
            self.claw.release(); self.rail.set_position(0); return False
        
        return True
    
    def approach(self, size, xbound, ybound, tgtProx, slwProx, angleTolerance, alpha, beta, detector):

        movement = True
        goal_x = goal_y = goal_th = None

        while movement:
            dist, th, glob_x, glob_y = self.vision(size, detector)
            
            # undo last movement if ball is lost
            if dist is None and not (goal_x == goal_y == goal_th == None):
                self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)
                goal_x = goal_y = goal_th = None; continue
            
            # out of bounds or no ball found
            if dist is None or glob_x > xbound or -glob_y > ybound:
                return False, None
            
            # early exit condition
            if dist < tgtProx and abs(th) < angleTolerance:
                break
            
            # smaller movements at close proximity
            beta_hat = 1 if dist < slwProx else beta
            alpha_hat = alpha / 2 if dist < slwProx else alpha
            
            # last minute angle correction 
            alpha_hat = 0 if dist < tgtProx else alpha_hat
            
            goal_x, goal_y, goal_th = self.transform(dist, th, alpha_hat, beta_hat)
            movement = self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)

        return True, dist
    
    def deliver(self, tgtProx, slwProx, angleTolerance, alpha, beta):

        x, y, _ = self.model.position()
        th = atan2(self.yBox, self.xBox)
        self.drive(goal_x=x, goal_y=y, goal_th=th)

        xbound = ybound = float('inf')
        detector = self.detector.find_box
        detected, dist = self.approach(self.boxHeight, xbound, ybound, tgtProx, slwProx, angleTolerance, alpha, beta, detector)

        while not detected:        
            self.explore(); detected, dist = self.approach(self.boxHeight, xbound, ybound, tgtProx, slwProx, angleTolerance, alpha, beta, detector)

        self.rail.set_position(1.0)
        x, y, th = self.transform(dist, th=0)
        self.drive(goal_x=x, goal_y=y, goal_th=th)
        
        self.claw.release()  

        x, y = x - 0.4, y + 0.4
        self.drive(goal_x=x, goal_y=y, goal_th=th)
        self.rail.set_position(0)

        return True
    
    def explore(self, reset=False):

        if reset:
            self.exploreState = 0; return None
        
        x, y, th = self.model.position()
        dec = 2*pi / 8 if self.exploreState < 8 else 2*pi / 16
        self.drive(goal_x = x, goal_y = y, goal_th = th - dec)

        self.exploreState = self.exploreState + 1
    
    def start(self, onTime, maxCamBallDist, maxCamBoxDist, grabDist, boxAngleTolerance, ballAngleTolerance, alphaReductionDist, alpha, beta):
        
        start_time = time()
        while time() - start_time < onTime:
            
            detector = self.detector.find_ball
            detected, dist = self.approach(self.ballRadius, self.xBound, self.yBound, maxCamBallDist, alphaReductionDist, ballAngleTolerance, alpha, beta, detector)

            if not detected:
                self.explore(); continue

            self.explore(reset=True)            
            
            collected = self.collect(dist, grabDist, alpha=alpha/2)
            
            if not collected:
                continue
            
            # delivered = self.deliver(maxCamBoxDist, alphaReductionDist, boxAngleTolerance, alpha, beta)

            self.rail.set_position(1)
            self.claw.release()
            self.rail.set_position(0)
            
            self.explore(reset=True)

        home = self.home()
        self.drive(*home)