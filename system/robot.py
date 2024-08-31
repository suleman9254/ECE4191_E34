from math import pi, cos, sin, atan2

class Robot(object):
    def __init__(self, model, controller, planner, camera=None, detector=None):

        self.camera = camera
        self.detector = detector
        self.model = model
        self.controller = controller
        self.planner = planner

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

    def vision(self, r_m):
        dist, th = None, None
        global_x, global_y = None, None

        frame = self.camera.read_frame()
        frame = self.camera.undistort(frame)
        frame, centroid, r_px = self.detector.find_ball(frame)
        
        if len(centroid):
            dist, th = self.camera.distance(*centroid, r_px, r_m)
            global_x, global_y, _ = self.transform(dist, th)

        return dist, th, global_x, global_y
    
    def collect(self, r_m, xbound, ybound, too_close, alpha, beta):
        dist, th, global_x, global_y = self.vision(r_m)
        
        if dist is not None:
            if dist > too_close:
                if abs(global_x) < xbound and abs(global_y) < ybound:

                    goal_x, goal_y, goal_th = self.transform(dist, th, alpha, beta)
                    self.drive(goal_x=goal_x, goal_y=goal_y, goal_th=goal_th)

                    return True
            else:
                
                goal_th = atan2(-self.model.y, -self.model.x)
                self.drive(self.model.x, self.model.y, goal_th)
                self.drive(goal_x=0, goal_y=0, goal_th=goal_th)

        return False
    
    def explore(self, dth):
        if (self.model.th + dth) // (2 * pi) > self.model.th // (2 * pi):
            dth = dth / 2
        
        self.drive(self.model.x, self.model.y, self.model.th + dth)
        return dth
    
    def start(self, r_m, alpha, beta, xbound, ybound, too_close, dth):
        while True:
            detected = self.collect(r_m, xbound, ybound, too_close, alpha, beta)

            if not detected:
                dth = self.explore(dth)
