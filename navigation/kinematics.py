import math
from threading import Thread

class DiffDriveModel(object):
    def __init__(self, motorL, motorR, wheel_radius=0.05, wheel_sep=0.15, dt=0.08, encoder_delay=0.04):
        
        self.x, self.y, self.th = 0, 0, 0
        self.wl, self.wr, self.v, self.w = 0, 0, 0, 0
        
        self.dt = dt
        self.l = wheel_sep
        self.r = wheel_radius
        self.enc_dt = encoder_delay

        self.motorL = motorL
        self.motorR = motorR
    
    def velocities(self):
        return self.wl, self.wr, self.v, self.w

    def position(self):
        return self.x, self.y, self.th
    
    def update_velocities(self):
        self.wl = self.motorL.read_velocity(self.enc_dt)
        self.wr = self.motorR.read_velocity(self.enc_dt)
        self.v = (self.wl*self.r - self.wr*self.r) / 2.0
        self.w = (self.wl*self.r + self.wr*self.r) / self.l
    
    def update_position(self):
        self.x = self.x + self.dt*self.v*math.cos(self.th)
        self.y = self.y + self.dt*self.v*math.sin(self.th)
        self.th = self.th + self.w*self.dt
    
    def brake(self):
        self.motorL.set_dir(None)
        self.motorR.set_dir(None)

    def set_pwm(self, duty_cycle_l, duty_cycle_r):
        self.motorL.set_pwm(duty_cycle_l)
        self.motorR.set_pwm(duty_cycle_r)

    def pose_update(self, duty_cycle_l, duty_cycle_r):
        self.set_pwm(duty_cycle_l, duty_cycle_r)
        self.update_velocities()
        self.update_position()
