import math
from threading import Thread

class DiffDriveModel(object):
    def __init__(self, motor_l, motor_r, wheel_radius=0.05, wheel_sep=0.15, dt=0.08, encoder_delay=0.04):
        
        self.x = 0.0 # x-position
        self.y = 0.0 # y-position 
        self.th = 0.0 # orientation
        
        self.wl = 0.0 # rotational velocity left wheel
        self.wr = 0.0 # rotational velocity right wheel

        self.v = 0.0 # forward velocity
        self.w = 0.0 # angular velocity
        
        self.r = wheel_radius
        self.l = wheel_sep
        self.dt = dt

        self.right_motor = motor_r
        self.left_motor = motor_l

        self.encoder_delay = encoder_delay

    def update_wheel_velocities(self):
        self.wl = self.left_motor.read_velocity(self.encoder_delay)
        self.wr = self.right_motor.read_velocity(self.encoder_delay)

    def update_base_velocities(self):
        self.v = (self.wl*self.r - self.wr*self.r) / 2.0
        self.w = (self.wl*self.r + self.wr*self.r) / self.l
    
    def update_position(self):
        self.x = self.x + self.dt*self.v*math.cos(self.th)
        self.y = self.y + self.dt*self.v*math.sin(self.th)
        self.th = self.th + self.w*self.dt

    def change_pwm(self, duty_cycle_l, duty_cycle_r):
        self.left_motor.change_pwm(duty_cycle_l)
        self.right_motor.change_pwm(duty_cycle_r)

    def pose_update(self, duty_cycle_l, duty_cycle_r):
        
        self.change_pwm(duty_cycle_l, 
                        duty_cycle_r)
        
        self.update_wheel_velocities()
        self.update_base_velocities()
        self.update_position()
