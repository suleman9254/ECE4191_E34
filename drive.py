from vision.camera import Camera
from vision.stream import Stream
from vision.detection import Detector

from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController
from navigation.planner import TentaclePlanner

from robot import Robot

from time import time
from math import atan

camera = Camera(cam_idx=0)
detector = Detector(minDist=50)
stream = Stream(host='0.0.0.0', port=5000)

max_count = 48 * 75
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=5, encB=6, max_count=max_count, dt=0.1)
motorL = Motor(in1=23, in2=24, enable=26, freq=100, encA=13, encB=19, max_count=max_count, dt=0.1)

# wheel rad and wheel sep ??
model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt=0.1, wheel_radius=0.05, wheel_sep=0.15)
controller = PIController(Kp=0.1, Ki=0.01, wheel_radius=0.05, wheel_sep=0.15)
planner = TentaclePlanner(dt=0.1, steps=5, alpha=1, beta=0.1)

robot = Robot(camera, detector, model, controller, planner)

vision_delay = 2 # seconds
max_dist = 5 # cm
r_cm = 3.35 # cm, ball radius
bounds_x = 411 # cm
bounds_y = 640 # cm

stream.start()

while True:
    goal_x, goal_y, goal_th = robot.find_ball(r_cm, stream)

    if goal_x is not None:
        if abs(goal_x) < bounds_x:
            if abs(goal_y) < bounds_y:
                
                if goal_x - robot.model.x > max_dist:
                    start_time = time()
                    while time - start_time < vision_delay:
                        robot.drive_to(goal_x, goal_y, goal_th)
                
                else:
                    robot.drive_to(goal_x=0, goal_y=0, goal_th=0)