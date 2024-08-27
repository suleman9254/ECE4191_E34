from vision.camera import Camera
from vision.stream import Stream
from vision.detection import Detector

from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController
from navigation.planner import TentaclePlanner

from system.robot import Robot

camera = Camera(cam_idx=0)
detector = Detector(minDist=50)
stream = Stream(host='0.0.0.0', port=5000)

max_count = 48 * 75
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)

model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt=0.1, wheel_radius=0.028, wheel_sep=0.22)
controller = PIController(Kp=0.1, Ki=0.01, wheel_radius=0.028, wheel_sep=0.22)
planner = TentaclePlanner(dt=0.1, steps=5, alpha=1, beta=0.1)

robot = Robot(camera, detector, model, controller, planner)

robot.detect_collect(r_m=0.0325, 
                     bounds_x=4.11, 
                     bounds_y=6.40, 
                     max_dist=0.1, 
                     drive_duration=2, 
                     home_tolerance=0.1)