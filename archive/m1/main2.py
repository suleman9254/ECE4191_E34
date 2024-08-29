from vision.camera import Camera
from vision.cht import Detector
from vision.yolo import YOLODetector

from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController
from navigation.planner import TentaclePlanner

from system.robot2 import Robot2

camera = Camera(cam_idx=0)
detector = YOLODetector(thresh=0.6)

max_count = 48 * 75
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)

model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt=0.08, wheel_radius=0.028, wheel_sep=0.22)
controller = PIController(Kp=0.1, Ki=0.01, wheel_radius=0.028, wheel_sep=0.22)
planner = TentaclePlanner(steps=5, alpha=1, beta=0.1, dt=0.08)

robot = Robot2(camera, detector, model, controller, planner)

r_m = 0.0325
max_dist = 0.2
xbound = 5.5
ybound = 4.11 

# robot.detect_collect(r_m, max_dist, dt=3, center_tolerance=80, xbound=xbound, ybound=ybound)

robot.middle(dt=10, xbound=xbound, ybound=ybound)