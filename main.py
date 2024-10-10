import numpy as np

from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController
from navigation.planner import TentaclePlanner

from transducers.stepper import Rail
from transducers.grabber import Claw
from transducers.sensors import Ultrasonic, ToFPanel

from vision.camera import Camera
from vision.detector import YOLODetector

from system.alt3 import Robot

max_count = 48 * 75
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)

claw = Claw(pin=18)
rail = Rail(motor_pins=[12, 16, 20, 21])

tof = ToFPanel(pins=[7, 8, 25])
# ultrasonic = Ultrasonic(trig=9, echo=11)
ultrasonic = Ultrasonic(trig=[9, 10], echo=[11, 14])

encoder_delay = 0.02
dt = 2 * encoder_delay + 0.00894649198
controller = PIController(Kp=0.15, Ki=0.02, wheel_radius=0.028, wheel_sep=0.22)
model = DiffDriveModel(motorL=motorL, motorR=motorR, dt=dt, wheel_radius=0.028, wheel_sep=0.22, encoder_delay=encoder_delay)

plannerSteps = 5*3
v, w = np.arange(-0.2, 0.2, 0.025), np.arange(-0.3, 0.3, 0.1)
planner = TentaclePlanner(v, w, steps=plannerSteps, alpha=1, beta=0.1, dt=dt)

box_ckpt, ball_ckpt = r'vision/ckpts/yolov11_box_V2.pt', r'vision/ckpts/yolov11_ball_V2.pt'
camera, detector = Camera(0), YOLODetector(box_ckpt=box_ckpt, ball_ckpt=ball_ckpt, ball_thresh=0.4, box_thresh=0.6)

params = {'xBox': 6.4, 'yBox': -4.11, 'xBound': 6.4, 'yBound': 4.11, 'ballRadius': 0.0325, 'boxHeight': 0.16}
robot = Robot(model, controller, planner, claw, rail, tof, ultrasonic, camera, detector, params)

onTime = 10 * 60

tgtBoxProx = (0.5, 0.3)
tgtBallProx = (0.5, 0.3)

deliverDist = 0.029
grabDist = np.array([0.11, 0.07, 0.11])

ballAngleTolerance = 0
boxAngleTolerance = 10 * 2 * np.pi / 360

beta = 0.8
alpha = 0.6

robot.start(onTime, tgtBallProx, tgtBoxProx, ballAngleTolerance, boxAngleTolerance, grabDist, deliverDist, alpha, beta)