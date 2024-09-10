from math import pi, cos, sin, atan2

class Robot(object):
    def __init__(self, model, controller, planner, claw, rail, sensor, camera=None, detector=None):

        self.caw = claw
        self.camera = camera
        self.detector = detector
        self.model = model
        self.controller = controller
        self.planner = planner
        self.sensor = sensor
        self.rail = rail

    def drive(self, goal_x, goal_y, goal_th):
        v, w = None, None
        while v != 0 or w != 0:
            v, w = self.planner.plan(goal_x, goal_y, goal_th, self.model.x, self.model.y, self.model.th)
            duty_cycle_l, duty_cycle_r = self.controller.drive(v, w, self.model.wl, self.model.wr)
            self.model.pose_update(duty_cycle_l, duty_cycle_r)
        
        self.model.pose_update(0, 0)
    
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
        
        if len(centroid):
            dist, th = self.camera.distance(*centroid, pixels, length)
            global_x, global_y, _ = self.transform(dist, th)

        return dist, th, global_x, global_y

    def home(self):
        goal_th = atan2(-self.model.y, -self.model.x)
        self.drive(self.model.x, self.model.y, goal_th)
        self.drive(goal_x=0, goal_y=0, goal_th=goal_th)
    
    def collect(self, grab_dist, retry_dist, success_dist, tries):
        while self.sensor.read() < retry_dist \
                and tries > 0:        

            self.claw.release()
            
            while self.sensor.read() > grab_dist:
                goal_x, goal_y, goal_th = self.transform(dist=self.sensor.read(), th=0, alpha=0.3, beta=0.8)
                self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)
        
            self.claw.grab()
            tries = tries - 1

            if self.sensor.read() < success_dist:
                return True
            
        return False
    
    def deliver(self):
        self.rail.set_position()
    
    def approach(self, r_m, xbound, ybound, start_collection_dist, alpha, beta, grab_dist, retry_dist, success_dist, tries):
        dist, th, global_x, global_y = self.vision(r_m, self.detector.find_ball)
        
        if dist is not None:
            if dist > start_collection_dist: # assuming origin @ bottom right corner
                if global_x < xbound and -global_y < ybound:

                    goal_x, goal_y, goal_th = self.transform(dist, th, alpha, beta)
                    self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)

                    return True
            else:
                
                collected = self.collect(grab_dist, retry_dist, success_dist, tries)
                # if collected:
                    # self.deliver()

        return False

    def explore(self, th, dth):        
        self.drive(self.model.x, self.model.y, self.model.th + dth)

        if (th + dth) // (2 * pi) > th // (2 * pi):
            th = th + dth
            dth = dth / 2
        else:
            th = th + dth
        
        return th, dth
    
    def start(self, r_m, alpha, beta, xbound, ybound, dth, start_collection_dist, grab_dist, retry_dist, success_dist, max_tries):
        th = 0

        while True:
            detected = self.collect(r_m, xbound, ybound, start_collection_dist, alpha, beta, grab_dist, retry_dist, success_dist, max_tries)

            if not detected:
                th, dth = self.explore(th, dth)
            else:
                th = 0
