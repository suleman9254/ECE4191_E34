from vision.camera import Camera
from vision.stream import Stream
from vision.detection import Detector

from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController
from navigation.planner import TentaclePlanner

from system.robot import Robot

import time
from threading import Timer

camera = Camera(cam_idx=0)
detector = Detector(minDist=50)
stream = Stream(host='0.0.0.0', port=5000)

max_count = 48 * 75
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=5, encB=6, max_count=max_count, dt=0.1)
motorL = Motor(in1=23, in2=24, enable=26, freq=100, encA=13, encB=19, max_count=max_count, dt=0.1)

model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt=0.1, wheel_radius=0.028, wheel_sep=0.22)
controller = PIController(Kp=0.1, Ki=0.01, wheel_radius=0.028, wheel_sep=0.22)
planner = TentaclePlanner(dt=0.1, steps=5, alpha=1, beta=0.1)

robot = Robot(camera, detector, model, controller, planner)

vision_delay = 2 # secs
max_dist = 0
r_m = 0.0325 # ball radius
bounds_x =  4.11
bounds_y = 6.40

def detect_and_collect(interval, stream=None):
    while True:
        frame, goal_x, goal_y, goal_th = robot.find_ball(r_m)
        
        if stream is not None:
            stream.set_frame(frame)

        if goal_x is not None:

            if abs(goal_x) < bounds_x:
                if abs(goal_y) < bounds_y:
                    
                    if abs(goal_x - robot.model.x) > max_dist:
                        
                        robot.drive_for_time(goal_x, goal_y, goal_th, dur=interval)
                    
                    else:
                        robot.drive_to_pos(goal_x=0, goal_y=0, goal_th=0, tolerance=0.2)
        
    return None

detect_and_collect(interval=vision_delay)
# stream.start()        

# detect_routine = Timer(vision_delay, 
#                        detect_and_collect, 
#                        args=(vision_delay,stream))
# detect_routine.start()