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

claw, ultrasonic = Claw(pin=18), DistanceSensor()
rail = Rail(motor_pins=[12, 16, 20, 21], start_pos=0, total_len=160)

encoder_delay = 0.02
dt = 2 * encoder_delay + 0.010424973
controller = PIController(Kp=0.05, Ki=0.02, wheel_radius=0.028, wheel_sep=0.22)
model = DiffDriveModel(motorL=motorL, motorR=motorR, dt=dt, wheel_radius=0.028, wheel_sep=0.22, encoder_delay=encoder_delay)

plannerSteps = 5*3
v, w = np.arange(-0.2, 0.3, 0.025), np.arange(-0.3, 0.3, 0.1)
planner = TentaclePlanner(v, w, steps=plannerSteps, alpha=1, beta=0.1, dt=dt)

box_ckpt, ball_ckpt = r'vision/ckpts/box_yolo11_new.pt', r'vision/ckpts/balls.pt'
camera, detector = Camera(0), YOLODetector(box_ckpt=box_ckpt, ball_ckpt=ball_ckpt, thresh=0.6)

params = {'xBox': 3, 'yBox': -2.2, 'xBound': 3, 'yBound': 2.2, 'ballRadius': 0.0325, 'boxHeight': 0.13}
robot = Robot(model, controller, planner, claw, rail, ultrasonic, camera, detector, params)

beta = 1
alpha = 1

grabDist = 0.04
maxCamBoxDist = 0.6
maxCamBallDist = 0.3
alphaReductionDist = 0.6

boxAngleTolerance = 0.2
ballAngleTolerance = 0

onTime = 5 * 60

robot.start(onTime, maxCamBallDist, maxCamBoxDist, grabDist, boxAngleTolerance, ballAngleTolerance, alphaReductionDist, alpha, beta)