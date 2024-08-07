import math

class Car(object):
    def __init__(self, motor_l, motor_r, dt=0.1, wheel_radius=0.05, wheel_sep=0.15):
        
        self.x = 0.0 # x-position
        self.y = 0.0 # y-position 
        self.th = 0.0 # orientation
        
        self.wl = 0.0 # rotational velocity left wheel
        self.wr = 0.0 # rotational velocity right wheel
        
        self.r = wheel_radius
        self.l = wheel_sep
        self.dt = dt

        self.right_motor = motor_r
        self.left_motor = motor_l
    
    # Veclocity motion model
    def base_velocity(self, wl, wr):
        
        v = (wl*self.r + wr*self.r) / 2.0
        w = (wl*self.r - wr*self.r) / self.l

        return v, w
    
    # Kinematic motion model
    def pose_update(self, duty_cycle_l, duty_cycle_r):

        self.wl = self.left_motor.change_pwm(duty_cycle_l)
        self.wr = self.right_motor.change_pwm(duty_cycle_r)
        
        v, w = self.base_velocity(self.wl, self.wr)
        
        self.x = self.x + self.dt*v*math.cos(self.th)
        self.y = self.y + self.dt*v*math.sin(self.th)
        self.th = self.th + w*self.dt

        return self.x, self.y, self.th