import numpy as np
from time import time
from math import cos, sin, atan2, pi

inf = float('inf')

class Robot(object):
    def __init__(self, model, controller, planner, claw=None, rail=None, tof=None, ultrasonic=None, camera=None, detector=None, params=None):

        self.claw = claw
        self.camera = camera
        self.detector = detector
        self.model = model
        self.controller = controller
        self.planner = planner
        self.ultrasonic = ultrasonic
        self.tof = tof
        self.rail = rail

        if params is not None:
            self.xBox = params['xBox']
            self.yBox = params['yBox']
            self.xBound = params['xBound']
            self.yBound = params['yBound']
            self.ballRadius = params['ballRadius']
            self.boxHeight = params['boxHeight']

        self.exploreState = 0

    def home(self):
        x, y, _ = self.model.position()
        self.drive((x, y, atan2(-y, -x)))
        self.drive((0, 0, atan2(-y, -x)))

    def reverse(self, dist):
        x, y, th = self.model.position()
        self.drive(goal=(x - dist, y + dist, th))
    
    def drive(self, goal, stallTime=1):

        timeout = False
        moveTime, vBaseTgt = time(), (1, 1)
        vWheel, vBase = self.model.velocities()
        pos = posInit = self.model.position()

        # till wheels stop spinning and planner agrees to stop
        while not {*vBase, *vBaseTgt} == {0} and not timeout:

            vBaseTgt = self.planner.plan(*goal, *pos)
            duty = self.controller.drive(*vBaseTgt, *vWheel)
            self.model.pose_update(*duty)

            pos = self.model.position()
            vWheel, vBase = self.model.velocities()

            if vWheel != (0, 0):
                moveTime = time()

            # early exit is stationary for stallTime
            timeout = time() - moveTime > stallTime

        self.model.brake()

        # True if movement
        return pos != posInit
        
    def transform(self, dist, th, alpha=1, beta=1):
        
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
    
    def cameraDrive(self, size, tgtProx, thTol, alpha, beta, detector, xBound=inf, yBound=inf):
        
        movement, goal = True, (None, None, None)
        atDest, (minProx, maxProx) = False, tgtProx

        while movement and not atDest:
            
            dist, th = self.vision(size, detector)

            # undo movement if ball got lost
            wasDetected = goal != (None, None, None)
            if dist is None and wasDetected: 
                self.drive(goal); goal = (None, None, None)
                continue
            
            # no detection or out of bounds
            if dist is None: return False
            globX, globY, _ = self.transform(dist, th)
            if globX > xBound or -globY > yBound: return False
            
            # early loop exit condition
            atDest = dist < minProx and abs(th) < thTol
            if atDest: continue
                        
            # last minute angle correction
            betaHat = 1 if dist < minProx else beta
            alphaHat = 0 if dist < minProx else alpha

            # drive to dist - maxProx; avoid overshoot
            goal = self.transform(dist - maxProx, th, alphaHat, betaHat)
            movement = self.drive(goal)

        goal = self.transform(dist, th)
        _ = self.drive(goal)
        
        return True
    
    def sensorDrive(self, sensor, dist, alpha, maxTime):

        startTime = time()
        meas = sensor.read()
        shouldRun = far = np.all(meas > dist)

        while shouldRun:
            
            # read sensor
            meas = sensor.read(); movDist = np.min(meas)
            x, y, th = self.transform(movDist, th=0, alpha=alpha)
            
            # out of bounds
            if x > self.xBound * 1.1 or -y > self.yBound * 1.1: break
            
            # move and read sensor again
            movement = self.drive(goal=(x, y, th))
            newMeas = sensor.read(); newMovDist = np.min(newMeas)

            # all sensors read too far
            far = np.all(newMeas > dist)

            # time is over
            timeout = time() - startTime > maxTime

            # dist decreased
            gotCloser = newMovDist < movDist * (1 - alpha / 2)
            
            # update shouldRun
            shouldRun = far and movement and gotCloser and not timeout
    
    def collect(self, grabDist, sensorTimeout, alpha):
        
        self.sensorDrive(grabDist, alpha, sensorTimeout, self.tof)
        
        self.claw.grab()

        # after grabbing
        meas = self.tof.read()
        if meas[1] > grabDist[1] * 1.2:
            self.claw.release()
            return False
        
        self.rail.set_position(0.1)

        # after raising
        meas = self.tof.read()
        if meas[1] < grabDist[1] * 1.2:
            return True

        self.claw.release()
        self.rail.set_position(0)
        return False
    
    def deliver(self, camProx, sonicProx, sensorTimeout, thTol, alpha, beta):

        # drive close to box with odom.
        x, y, _ = self.model.position()
        th = atan2(self.yBox, self.xBox)
        x, y = max(x, self.xBox - 1), min(y, self.yBox + 1)
        
        self.drive(goal=(x, y, th))

        # vision to correct angle and approach
        approached = self.cameraDrive(self.boxHeight, camProx, thTol, alpha, beta, self.detector.find_box)
        while not approached:
            self.explore(); approached = self.cameraDrive(self.boxHeight, camProx, thTol, alpha, beta, self.detector.find_box)
        
        self.rail.set_position(1)

        # drive close with ultrasonics
        self.sensorDrive(sonicProx, alpha, sensorTimeout, self.ultrasonic)

        # release / reverse
        self.claw.release()
        self.reverse(dist=0.6)
        self.rail.set_position(0)

    def explore(self, reset=False):

        if reset: self.exploreState = 5; return None

        x, y, th = self.model.position()
        
        if self.exploreState < 4:
            th = th - 2 * pi / 8
        
        elif self.exploreState < 5:
            th = atan2(self.yBox, self.xBox)
            x, y = self.xBox / 2, self.yBox / 2

        elif self.exploreState < 6:
            th = th - 2 * pi / 8
        
        elif self.exploreState < 14:
            th = th - 2 * pi / 16

        elif self.exploreState < 30:
            self.exploreState = 6

        self.drive(goal=(x, y, th))
        self.exploreState = self.exploreState + 1
    
    def start(self, onTime, tgtBallProx, tgtBoxProx, thBallTol, thBoxTol, grabDist, deliverDist, alpha, beta, collectSensorTimeout=5, deliverSensorTimeout=inf):
        
        startTime = time()
        while time() - startTime < onTime:
            
            detected = self.cameraDrive(self.ballRadius, tgtBallProx, thBallTol, alpha, beta, self.detector.find_ball, self.xBound, self.yBound)

            if not detected: self.explore(); continue
            
            self.explore(reset=True)
            collected = self.collect(grabDist, collectSensorTimeout, alpha)
            
            _ , revDist = tgtBallProx
            if not collected: self.reverse(revDist); continue

            self.deliver(tgtBoxProx, deliverDist, deliverSensorTimeout, thBoxTol, alpha, beta)
        
        self.home()