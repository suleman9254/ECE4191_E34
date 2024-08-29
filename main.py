from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController
from navigation.planner import TentaclePlanner

from system.robot import Robot

import numpy as np

max_count = 48 * 75
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)

encoder_delay = 0.02
dt = 2 * encoder_delay + 0.005
controller = PIController(Kp=0.025, Ki=0.001, wheel_radius=0.028, wheel_sep=0.22)
model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt=dt, wheel_radius=0.028, wheel_sep=0.22, encoder_delay=encoder_delay)

v = np.arange(-0.2, 0.5, 0.1)
w = np.arange(-1, 1, 0.125)
planner = TentaclePlanner(v, w, steps=5, alpha=1, beta=0.1, dt=dt)

from math import pi

robot = Robot(model, controller, planner)

while True:
    robot.drive(0, 0, robot.model.th + 0.1)