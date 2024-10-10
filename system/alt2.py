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
    
    def _transform(self, dist, th, alpha=1, beta=1):
        
        dist, th = alpha * dist, beta * th
        local_x, local_y = dist * cos(th), dist * sin(th)

        global_x = self.model.x + local_x * cos(self.model.th) - local_y * sin(self.model.th)
        global_y = self.model.y + local_x * sin(self.model.th) + local_y * cos(self.model.th)
        
        global_th = self.model.th + th

        return global_x, global_y, global_th
    
    def drive(self, goal, stallTime=0.2):

        vBaseTgt = (0, 0)
        moveTime, timeout = time(), False

        vWheel, vBase = self.model.velocities()
        pos = posInit = self.model.position()

        # till wheels stop spinning and planner agrees to stop
        while not {*vBase, *vBaseTgt} == {0} and not timeout:

            vBaseTgt = self.planner.plan(*goal, *pos)
            duty = self.controller.drive(*vBaseTgt, *vWheel)

            # bypass controller to brake
            brake = np.all(vBaseTgt == 0)
            self.model.brake() if brake else self.model.set_pwm(*duty)

            # update model position
            self.model.pose_update()
            pos = self.model.position()
            vWheel, vBase = self.model.velocities()

            # early exit is stationary for stallTime
            if vWheel != (0, 0): moveTime = time()
            timeout = time() - moveTime > stallTime

        self.model.brake()
        return pos != posInit

    def vision(self, length, detector):
        
        frame = self.camera.read_frame()
        frame = self.camera.undistort(frame)
        frame, centroid, pixels = detector(frame)
        
        dist, th = None, None
        if centroid is not None:
            dist, th = self.camera.distance(*centroid, pixels, length)

        return dist, th
    
    def approach(self, size, tgtProx, thTol, alpha, beta, detector, xBound=inf, yBound=inf):
        
        movement, goal = True, None
        atDest, (minProx, maxProx) = False, tgtProx

        while movement and not atDest:
            
            dist, th = self.vision(size, detector)

            # undo movement if ball got lost
            wasDetected = goal != None
            if dist is None and wasDetected: 
                self.drive(goal=goal); goal=None; continue
            
            # no detection
            if dist is None: return False
            
            # out of bounds
            globX, globY, _ = self._transform(dist, th)
            if globX > xBound or -globY > yBound: return False
            
            # early loop exit condition
            atDest = dist < minProx and abs(th) < thTol
            if atDest: continue
                        
            # last minute angle correction
            betaHat = 1 if dist < minProx else beta
            alphaHat = 0 if dist < minProx else alpha

            # drive to dist - maxProx; avoid overshoot
            goal = self._transform(dist - maxProx, th, alphaHat, betaHat)
            movement = self.drive(goal=goal)

        goal = self._transform(dist, th)
        _ = self.drive(goal=goal)
        
        return True
    
    def _collectDrive(self, grabDist, collectTimeout, alpha):
        
        startTime = time()
        meas = self.tof.read()
        
        while np.all(meas > grabDist) and \
                time() - startTime < collectTimeout:

            goal = self._transform(dist=np.min(meas), th=0, alpha=alpha)
            self.drive(goal=goal); meas = self.tof.read()

    def collect(self, grabDist, collectTimeout, alpha):
        
        self._collectDrive(grabDist, collectTimeout, alpha)

        self.claw.grab()

        _, lost, _ = self.tof.read() > self.grabDist * 1.2
        if lost: self.claw.release(); return False

        self.rail.set_position(0.1)

        _, lost, _ = self.tof.read() > self.grabDist * 1.2
        if not lost: return True

        self.claw.release()
        self.rail.set_position(0)
        return False
    
    def _turn(self, th):
        x_, y_, th_ = self.model.position()
        self.drive(goal = (x_, y_, th_ - th))

    def _deliveryTurn(self, thTol, beta, maxTime):

        l, r = self.ultrasonic.read()
        th = atan2(r - l, 0.13)
        startTime = time()

        while abs(th) < thTol and \
                time() - startTime < maxTime:
            
            self._turn(th * beta); l, r = self.ultrasonic.read(); th = atan2(r - l, 0.13)
        
    def _deliveryDrive()

    def deliver(self, camProx, deliverDist, deliverTimeout, thTol, alpha, beta):
        
        # drive close to box with odom.
        x, y, _ = self.model.position()
        th = atan2(self.yBox, self.xBox)
        self.drive((x, y, th))
        
        minProx, maxProx = camProx
        x = max(x, self.xBox - minProx)
        y = min(y, self.yBox + minProx)
        self.drive((x, y, th))

        # vision to correct angle and approach
        start_time, timeout = time(), False
        approached = self.cameraDrive(self.boxHeight, camProx, thTol, alpha, beta, self.detector.find_box)
        
        while not approached and not timeout:
            self.explore(); timeout = time() - start_time < deliverTimeout
            approached = self.cameraDrive(self.boxHeight, camProx, thTol, alpha, beta, self.detector.find_box)
        
        # drive close with ultrasonics
        self.rail.set_position(1)

        self._deliveryTurn(thTol, beta, deliverTimeout)
        self._deliveryDrive(deliverDist)

        self.claw.release()
        self._reverse(dist=minProx)
        self.rail.set_position(0)
        self._turn(2 * pi / 4)
    

    def _driveWithUltrasonic(self):

        meas = self.ultrasonic.read(); movDist = np.min(meas)
        goal = self._transform(movDist, th=0, alpha=0.5)

        fin = movDist < self.deliverDist
        return None if fin else goal

    def _reverse(self, dist):
        x, y, th = self.model.position()
        self.drive(goal = (x - dist, y + dist, th))

    def deliver(self, boxProx, boxAngleTol, deliverDist):
        
        x, y, _ = self.model.position()
        self.drive(goal = (x, y, -pi/2))

        self.drive(goal = (x, self.yBox, -pi/2))
        self.drive(goal = (x, self.yBox, 0))
        self.drive(goal = (self.xBox - boxProx, self.yBox, 0))

        x, y, _ = self.model.position()
        th = atan2(self.yBox, self.xBox)
        self.drive(goal = (x, y, th))

        x = max(x, self.xBox - boxProx)
        y = min(y, self.yBox + boxProx)
        self.drive(goal = (x, y, th))

        meas = self.ultrasonic.read()
        tooFar = np.all(meas > 0.9)
        
        while tooFar: 
            self._turn(2 * pi / 8)
            meas = self.ultrasonic.read()
            tooFar = np.all(meas > 0.9)

        self.rail.set_position(1)

        self.boxAngleTol = boxAngleTol
        self.drive(goalComputeFn = self._turnWithUltrasonic)

        self.deliverDist = deliverDist
        self.drive(goalComputeFn = self._driveWithUltrasonic)

        self.claw.release()
        self._reverse(dist=boxProx * 4)
        self.rail.set_position(0)
        self._turn(2 * pi / 4)

    def explore(self, reset=False):

        if reset: self.exploreState = 5; return None

        (x, y, th) = (x0, y0, th0) = self.model.position()
        
        if self.exploreState < 4:
            th = th - 2 * pi / 16
        
        elif self.exploreState < 5:
            th = atan2(self.yBox, self.xBox)
            x, y = self.xBox / 2, self.yBox / 2

        elif self.exploreState < 6:
            th = th - 2 * pi / 8
        
        elif self.exploreState < 14:
            th = th - 2 * pi / 16

        elif self.exploreState < 30:
            self.exploreState = 6

        self.drive(goal=(x0, y0, th)); self.drive(goal=(x, y, th))
        self.exploreState = self.exploreState + 1
    
    def home(self):
        x, y, _ = self.model.position()
        th = atan2(-y, -x)

        self.drive(goal = (x, y, th))
        self.drive(goal = (0, 0, th))
    
    def start(self, onTime, tgtBallProx, boxProx, thBallTol, thBoxTol, grabDist, deliverDist, alpha, beta, collectTimeout=5):

        startTime = time()
        while time() - startTime < onTime:
            
            try:
                detected = self.approach(self.ballRadius, tgtBallProx, thBallTol, alpha, beta, self.detector.find_ball, self.xBound, self.yBound)

                if not detected: self.explore(); continue
                
                self.explore(reset=True)
                collected = self.collect(grabDist, collectTimeout)
                
                _ , revDist = tgtBallProx
                if not collected: self._reverse(revDist); continue

                self.deliver(boxProx, thBoxTol, deliverDist)
                self.explore(reset=True)
            
            except KeyboardInterrupt:
                break
        
        self.home()