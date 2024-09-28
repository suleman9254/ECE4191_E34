from time import time
from math import cos, sin, atan2

class Robot(object):
    def __init__(self, model, controller, planner, claw=None, rail=None, sensor=None, camera=None, detector=None):

        self.claw = claw
        self.camera = camera
        self.detector = detector
        self.model = model
        self.controller = controller
        self.planner = planner
        self.sensor = sensor
        self.rail = rail

        self.timeout = 1

    def home(self):
        x, y, _ = self.model.position()
        return 0, 0, atan2(-y, -x)
    
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
        return True if (x, y, th) != (x0, y0, th0) else False
        
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
    
    def collect(self, grab_dist, alpha):
        
        movement = True
        distance = self.sensor.read()

        while distance > grab_dist and movement:
            
            goal_x, goal_y, goal_th = self.transform(dist=distance, th=0, alpha=alpha)
            movement = self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)
            distance = self.sensor.read()

        self.claw.grab()

        if self.sensor.read() < grab_dist * 1.1:
            return True

        self.claw.release()
        return False
    
    def approach(self, r_m, xbound, ybound, d_lim, alpha, beta, detector):
        
        movement = True
        goal_x = goal_y = goal_th = None

        while movement:
            dist, th, glob_x, glob_y = self.vision(r_m, detector)

            # undo last movement if ball is lost
            if dist is None and not (goal_x == goal_y == goal_th == None):
                self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)
                goal_x = goal_y = goal_th = None
                continue
            
            # out of bounds or no ball found
            if dist is None or glob_x > xbound or -glob_y > ybound:
                return False

            # last minute angle correction 
            dist = 0 if dist < d_lim else dist

            goal_x, goal_y, goal_th = self.transform(dist, th, alpha, beta)
            movement = self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)

        return True
    
    # def deliver(self, x, y, height, d_lim, th_lim, alpha, beta):

    #     self.rail.set_position(0.3)

    #     goal_th = atan2(x, y)
    #     goal_x, goal_y = x * 0.5, y * 0.5
    #     self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)

    #     inf = 10000
    #     detected = self.approach(height, inf, inf, d_lim, th_lim, alpha, beta, self.detector.find_box)
    #     while not detected:

    # def explore(self):
    
    def start(self, op_time, r_m, alpha, beta, xbound, ybound, collect_dist, grab_dist):
        
        start_time = time()
        while time() - start_time < op_time:

            detected = self.approach(r_m, xbound, ybound, collect_dist, alpha, beta, self.detector.find_ball)

            if not detected:
                self.explore()
                continue
                        
            collected = self.collect(grab_dist, alpha)

            if collected:
                self.deliver()

        home = self.home()
        self.drive(*home)

                


