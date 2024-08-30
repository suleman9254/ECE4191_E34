from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController
from navigation.planner import TentaclePlanner

from vision.camera import Camera
from vision.yolo_detector import YOLODetector

from system.robot import Robot
import numpy as np

max_count = 48 * 75
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)

encoder_delay = 0.04
dt = 2 * encoder_delay + 0.005
controller = PIController(Kp=0.05, Ki=0.002, wheel_radius=0.028, wheel_sep=0.22)
model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt=dt, wheel_radius=0.028, wheel_sep=0.22, encoder_delay=encoder_delay)

v = np.arange(-0.2, 0.4, 0.1)
w = np.arange(-1, 1, 0.25)
planner = TentaclePlanner(v, w, steps=5, alpha=1, beta=0.1, dt=dt)

camera = Camera(0)
detector = YOLODetector(path='vision/yolo.pt')

robot = Robot(model, controller, planner, camera=camera, detector=detector)

from math import pi
# robot.drive(0, 0, -pi/2)

print(robot.model.th)

robot.start(r_m=0.0325, xbound=1000000, ybound=10000000, too_close=0.2, beta=0.8, alpha=0.5)