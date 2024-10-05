import numpy as np
from time import time
from math import cos, sin, atan2, pi

from statistics import mean

inf = float('inf')

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
        self.driveTimeout = 0.2
        self.collectTimeout = 10
    
    def drive(self, goal):

        timeout = False
        moveTime, vBaseTgt = time(), (1, 1)
        vWheel, vBase = self.model.velocities()
        pos = posInit = self.model.position()

        while not {*vBase, *vBaseTgt} == {0} and not timeout:

            vBaseTgt = self.planner.plan(*goal, *pos)
            duty = self.controller.drive(*vBaseTgt, *vWheel)
            self.model.pose_update(*duty)

            pos = self.model.position()
            vWheel, vBase = self.model.velocities()

            if vWheel != (0, 0):
                moveTime = time()

            timeout = time() - moveTime > self.driveTimeout

        self.model.brake()
        movement = pos != posInit
        return movement
        
    def transform(self, dist, th, alpha=1, beta=1):
        
        if dist is None or th is None:
            return inf, inf, inf

        dist, th = alpha * dist, beta * th
        local_x, local_y = dist * cos(th), dist * sin(th)

        global_x = self.model.x + local_x * cos(self.model.th) - local_y * sin(self.model.th)
        global_y = self.model.y + local_x * sin(self.model.th) + local_y * cos(self.model.th)
        
        global_th = self.model.th + th

        return global_x, global_y, global_th

    def vision(self, length, detector):
        
        frame = self.camera.read_frame()
        frame = self.camera.undistort(frame)
        frame, centroid, pixels = detector(frame)
        
        dist, th = None, None
        if centroid is not None:
            dist, th = self.camera.distance(*centroid, pixels, length)

        return dist, th
    
    def approach(self, size, tgtProx, thTol, alpha, beta, detector, bounds):
        
        xBound, yBound = bounds
        atDest, (minProx, maxProx) = False, tgtProx
        movement, goal = True, (None, None, None)

        while movement and not atDest:
            
            dist, th = self.vision(size, detector)
            globX, globY, _ = self.transform(dist, th)

            wasDetected = goal != (None, None, None)
            if dist is None and wasDetected: 
                self.drive(goal); goal = (None, None, None)
                continue

            outBound = globX > xBound or -globY > yBound
            if dist is None or outBound: return False
            
            atDest = dist < minProx and abs(th) < thTol
            if atDest: continue

            gDist = dist - maxProx
            betaHat = 1 if dist < minProx else beta
            alphaHat = 0 if dist < minProx else alpha

            goal = self.transform(gDist, th, alphaHat, betaHat)
            movement = self.drive(goal)

        goal = self.transform(dist, th)
        _ = self.drive(goal)
        
        return True
    
    def collect(self, grabDist, alpha):
        
        startTime = time()
        far = self.sensor.read() > grabDist
        movement, lost, timeout = True, False, False

        while far and movement and not lost and not timeout:
            
            dist = self.sensor.read()
            goal = self.transform(dist, th=0, alpha=alpha)
            movement = self.drive(goal)

            far = self.sensor.read() > grabDist
            lost = self.sensor.read() > dist * 1.2
            timeout = time() - startTime > self.collectTimeout
        
        self.claw.grab()

        if self.sensor.read() > grabDist * 1.2:
            self.claw.release()
            return False
        
        self.rail.set_position(0.1)

        if self.sensor.read() < grabDist * 1.2:
            return True

        self.claw.release()
        self.rail.set_position(0)
        return False
    
    def explore(self, reset=False):

        if reset: self.exploreState = 0; return None
        
        x, y, th = self.model.position()
        dec = 2*pi / 8 if self.exploreState < 8 else 2*pi / 16
        self.drive((x,y,th-dec))

        self.exploreState = self.exploreState + 1
    
    def start(self, onTime, tgtBallProx, thBallTol, grabDist, alpha, beta):
        
        start_time = time()
        while time() - start_time < onTime:
            
            detector = self.detector.find_ball
            bounds = (self.xBound, self.yBound)
            detected = self.approach(self.ballRadius, tgtBallProx, thBallTol, alpha, beta, detector, bounds)

            if not detected: self.explore(); continue

            self.explore(reset=True)            
            
            collected = self.collect(grabDist, alpha)

            if collected:
                self.rail.set_position(0.6)
                self.claw.release()
                self.rail.set_position(0)
            
            self.explore(reset=True)