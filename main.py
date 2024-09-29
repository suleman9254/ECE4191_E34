import numpy as np

from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController
from navigation.planner import TentaclePlanner

from transducers.grabber import Claw
from transducers.sensor import DistanceSensor
from transducers.stepper import Rail

from vision.camera import Camera
from vision.detector import YOLODetector

from system.robot import Robot

max_count = 48 * 75
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)

claw, ultrasonic = Claw(pin=18), None
rail = Rail(motor_pins=[12, 16, 20, 21], start_pos=0)

encoder_delay = 0.02
dt = 2 * encoder_delay + 0.00127
controller = PIController(Kp=0.05, Ki=0.02, wheel_radius=0.028, wheel_sep=0.22)
model = DiffDriveModel(motorL=motorL, motorR=motorR, dt=dt, wheel_radius=0.028, wheel_sep=0.22, encoder_delay=encoder_delay)

v, w = np.arange(-0.2, 0.2, 0.1), np.arange(-0.3, 0.3, 0.1)
planner = TentaclePlanner(v, w, steps=5, alpha=1, beta=0.1, dt=dt)

box_ckpt, ball_ckpt = r'vision/ckpts/boxes.pt', r'vision/ckpts/balls_old.pt'
camera, detector = Camera(0), YOLODetector(box_ckpt=box_ckpt, ball_ckpt=ball_ckpt, thresh=0.6)

params = {'xBox': 1, 'yBox': -1, 'xBound': 6.4, 'yBound': 4.11, 'ballRadius': 0.0325, 'boxHeight': 0.13}
robot = Robot(model, controller, planner, claw, rail, ultrasonic, camera, detector, params)

beta = 0.8
alpha = 0.6
grabDist = 0.01
onTime = 10 * 60
maxCamBallDist = 0.3
maxCamBoxDist = 0.3 * 2
alphaReductionDist = 0.6

robot.start(onTime, maxCamBallDist, maxCamBoxDist, grabDist, alphaReductionDist, alpha, beta)