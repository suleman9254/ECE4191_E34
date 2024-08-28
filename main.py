from vision.camera import Camera
from vision.cht import Detector

from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController
from navigation.planner import TentaclePlanner

from system.robot import Robot

import numpy as np
from time import time, sleep

camera = Camera(cam_idx=0)
detector = Detector(minDist=50, hsv_low=[13, 50, 0], hsv_high=[30, 255, 255])

max_count = 48 * 75
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)

model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt_motion=0.08, dt_enc=0.04, wheel_radius=0.028, wheel_sep=0.22)
controller = PIController(Kp=0.1, Ki=0.01, wheel_radius=0.028, wheel_sep=0.22)
planner = TentaclePlanner(dt=0.08, steps=5, alpha=1, beta=0.1)

robot = Robot(camera, detector, model, controller, planner)

delay = 0.2
r_m = 0.0325
bounds_x = 4.11 / 2
bounds_y = 6.4 / 2
max_dist = 0.2

robot.detect_collect(r_m, bounds_x, bounds_y, max_dist, delay, tolerance)

# spirals = 3
# separation = 5
# resolution = 20
# x_center = bounds_x / 2
# y_center = bounds_y / 2

# th = np.linspace(0, spirals * 2 * np.pi, resolution)
# radii = separation * th / (2 * np.pi)

# x_vec = x_center + radii * np.cos(th)
# y_vec = y_center + radii * np.sin(th)

# start_time = time()

# for goal_x, goal_y, goal_th in zip(x_vec, y_vec, th):

#     robot.drive_step(goal_y, goal_x, goal_th)
#     sleep(5)

#         # if time() - start_time > delay:
#             # robot.model.pose_update(duty_cycle_l=0, duty_cycle_r=0)
#             # robot.detect_collect(r_m, bounds_x, bounds_y, max_dist, delay, tolerance)
#             # start_time = time()

# robot.model.pose_update(duty_cycle_l=0, duty_cycle_r=0)