from vision.camera import Camera
from vision.cht import Detector
from vision.yolo import YOLODetector

from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controllerv2 import PIController
from navigation.planner import TentaclePlanner

from system.robot3 import Robot3

camera = Camera(cam_idx=0)
detector = YOLODetector(thresh=0.6)

max_count = 48 * 75
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)

model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt=0.08, wheel_radius=0.028, wheel_sep=0.22)
controller = PIController(Kp=0.1, Ki=0.01, wheel_radius=0.028, wheel_sep=0.22)
planner = TentaclePlanner(steps=1, alpha=1, beta=0.1, dt=0.08)

robot = Robot3(camera, detector, model, controller, planner)

delay = 0.2
r_m = 0.0325
bounds_x = 4.11
bounds_y = 6.4
max_dist = 0.2
tolerance = 0.1

robot.detect_collect(r_m, max_dist, cntr_px_tol=60)

# robot.drive(goal_x=1, goal_y=0, goal_th=0, dt=3)
# robot.brake(dt=0.5)