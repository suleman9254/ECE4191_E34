import numpy as np
from vision.camera import Camera
from vision.yolo_detector import YOLODetector
from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController
from navigation.planner import TentaclePlanner
from system.robot import Robot
import cv2
import os
from time import sleep, time

class Align(object):
    def __init__(self, robot, frame_width, tolerance=10, output_dir='frames'):
        self.robot = robot
        self.frame_width = frame_width
        self.tolerance = tolerance
        self.output_dir = output_dir
        self.frame_count = 0

        # Ball parameters
        self.actual_diameter_m = (6.54 + 6.86) / 2 / 100  # Convert to meters
        
        # Load camera matrix
        self.camera_matrix = np.load('vision/params/camera_matrix.npy')
        self.dist_coeffs = np.zeros((5, 1))  # Assuming no distortion coefficients

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

    def align_to_center(self):
        """
        Continuously adjust the robot's orientation to align the detected ball to the center of the camera frame.
        """
        last_frame_time = time()
        while True:
            current_time = time()
            frame = self.robot.camera.read_frame()
            frame = self.robot.camera.undistort(frame)

            # Use YOLO detector to find the ball
            processed_frame, centroid, bbox = self.robot.detector.find_ball(frame)

            # Save processed frame
            filename = os.path.join(self.output_dir, f'frame_{self.frame_count:04d}.jpg')
            cv2.imwrite(filename, processed_frame)
            print(f"Frame saved as {filename}")
            self.frame_count += 1

            if centroid[0] is None or centroid[1] is None:
                print("No detection")
                # Turn a bit to search for the ball if not detected
                self.rotate_robot(0.2)
                sleep(0.2)
                continue

            x_ball = centroid[0]
            frame_center = self.frame_width / 2
            deviation = x_ball - frame_center
            print(f"Distance from center: {deviation}")

            # Calculate the distance to the ball
            distance = self.calculate_distance(bbox)
            print(f"Estimated distance to the ball: {distance:.2f} meters")

            # Calculate time delay
            frame_delay = current_time - last_frame_time
            print(f"Frame delay: {frame_delay:.2f} seconds")

            # Optionally adjust for frame delay
            # Adjustments can be based on your specific needs and the robot's movement

            if abs(deviation) <= self.tolerance:
                print("Object is centered.")
                self.stop_robot()
                break

            # Rotate robot based on the deviation
            rotation_speed = 0.2 if deviation > 0 else -0.2
            print(f"Rotating with speed: {rotation_speed}")
            self.rotate_robot(rotation_speed)

            last_frame_time = current_time
            sleep(0.2)

    def rotate_robot(self, rotation_speed):
        """
        Rotate the robot at a specified rotation speed.
        
        Parameters:
        - rotation_speed: The speed at which to rotate the robot.
        """
        # Ensure rotation speed is set correctly for left and right motors
        duty_cycle_l, duty_cycle_r = self.robot.controller.drive(0, rotation_speed, self.robot.model.r, self.robot.model.r)
        print(f"Duty Cycles - Left: {duty_cycle_l}, Right: {duty_cycle_r}")
        self.robot.model.pose_update(duty_cycle_l, duty_cycle_r)

    def stop_robot(self):
        """
        Stop the robot's movement.
        """
        self.robot.model.pose_update(0, 0)
        print("Robot stopped.")

    def calculate_distance(self, bbox):
        """
        Calculate the distance from the camera to the ball using the bounding box size.
        
        Parameters:
        - bbox: Bounding box coordinates of the detected ball in the frame.
        
        Returns:
        - Distance to the ball in meters.
        """
        if bbox is None or len(bbox) < 4:
            return None
        
        bbox_width_px = bbox[2] - bbox[0]
        
        # Assuming bbox_width_px is in pixels and actual_diameter_m is the real-world diameter of the ball
        focal_length_px = self.camera_matrix[0, 0]  # Using fx (focal length in pixels)
        distance = (self.actual_diameter_m * focal_length_px) / bbox_width_px
        
        return distance

# Initialize components and robot as in your original code
camera = Camera(cam_idx=0)
detector = YOLODetector(path='/home/g34/my_codebase/archive/RobitShit/best.pt', thresh=0.5)  # Update path if needed
max_count = 48 * 75
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)
model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt=0.08, wheel_radius=0.028, wheel_sep=0.22)
controller = PIController(Kp=0.2, Ki=0.05, wheel_radius=0.028)
robot = Robot(camera=camera, detector=detector, model=model, controller=controller, planner=None)

aligner = Align(robot=robot, frame_width=640, tolerance=10)
aligner.align_to_center()

