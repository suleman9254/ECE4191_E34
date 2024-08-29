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

    def vision(self, r_m):
        dist, th, x, y = None, None, None, None

        frame = self.camera.read_frame()
        frame = self.camera.undistort(frame)
        frame, centroid, r_px = self.detector.find_ball(frame)
        
        if len(centroid):
            dist, th = self.camera.distance(*centroid, r_px, r_m)
            x = dist * cos(th) + self.model.x
            y = dist * sin(th) + self.model.y
        
        return dist, th, x, y
    
    def collect(self, r_m, alpha, xbound, ybound, too_close):
        while True:
            
            dist, th, x, y = self.vision(r_m)
            if dist is not None:
                if dist < too_close:
                    if abs(x) < xbound and abs(y) < ybound:

                        goal_th = self.model.th + th * alpha
                        goal_x = self.model.x + dist * alpha
                        
                        self.drive(self.model.x, self.model.y, goal_th)
                        self.drive(goal_x, self.model.y, self.model.th)
                
                else:
                    goal_th = atan2(-self.model.y, -self.model.x)
                    goal_x, goal_y = 0, 0

                    self.drive(self.model.x, self.model.y, goal_th)
                    self.drive(goal_x, goal_y, self.model.th)

            else:
                
                goal_th = self.model.th + 0.1
                self.drive(self.model.x, self.model.y, goal_th)