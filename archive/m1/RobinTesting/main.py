from vision.camera import Camera
from vision.cht import Detector
from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController
from navigation.planner import TentaclePlanner
from system.robot import Robot

import numpy as np
import time
import cv2
import os
import sys
import math

class SquarePathController:
    def __init__(self, robot, side_length, linear_speed=0.1, angular_speed=0.5):
        """
        Initializes the square path controller for the robot.
        
        Parameters:
        - robot: The robot object containing the camera, detector, motors, model, and controller.
        - side_length: The length of each side of the square in meters.
        - linear_speed: The speed at which the robot moves forward (m/s).
        - angular_speed: The speed at which the robot turns (rad/s).
        """
        self.robot = robot
        self.side_length = side_length
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

    def move_forward(self, distance):
        """
        Moves the robot forward by a specified distance.
        
        Parameters:
        - distance: The distance to move forward in meters.
        """
        start_x, start_y, start_th = self.robot.model.x, self.robot.model.y, self.robot.model.th
        print(f"Starting move forward from ({start_x}, {start_y}) with orientation {start_th}")

        while True:
            current_x, current_y, current_th, _, _ = self.robot.model.pose_update(self.linear_speed, self.linear_speed)
            traveled_distance = math.sqrt((current_x - start_x) ** 2 + (current_y - start_y) ** 2)
            print(f"Traveled distance: {traveled_distance:.2f} meters")

            if traveled_distance >= distance:
                print("Reached the target distance.")
                break

            time.sleep(0.1)

        # Stop the robot
        self.robot.model.pose_update(0, 0)

    def turn_90_degrees(self):
        """
        Rotates the robot by 90 degrees.
        """
        start_th = self.robot.model.th
        print(f"Starting rotation from orientation {start_th}")

        while True:
            current_x, current_y, current_th, _, _ = self.robot.model.pose_update(-self.angular_speed, self.angular_speed)
            angle_turned = abs(current_th - start_th)

            print(f"Angle turned: {math.degrees(angle_turned):.2f} degrees")

            if angle_turned >= math.radians(90):
                print("Completed 90-degree turn.")
                break

            time.sleep(0.1)

        # Stop the robot
        self.robot.model.pose_update(0, 0)

    def execute_square_path(self):
        """
        Makes the robot move in a square path.
        """
        for _ in range(4):
            self.move_forward(self.side_length)
            self.turn_90_degrees()
        print("Completed square path.")

# Initialize components and robot as in your original code...
camera = Camera(cam_idx=0)
detector = Detector(minDist=50, hsv_low=[13, 50, 0], hsv_high=[30, 255, 255])
max_count = 48 * 75
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)
model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt=0.08, wheel_radius=0.028, wheel_sep=0.22)
controller = PIController(Kp=0.2, Ki=0.05, wheel_radius=0)
robot = Robot(camera=camera, detector=detector, model=model, controller=controller, planner=None)

# Create the square path controller
square_path_controller = SquarePathController(robot, side_length=1.0)  # Set the side length of the square to 1 meter

# Execute the square path
square_path_controller.execute_square_path()
