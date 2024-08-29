import numpy as np
import cv2
import time
from ultralytics import YOLO
import numpy as np


class RobotPose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def get_pose(self):
        return self.x, self.y, self.theta

    def move(self, dx, dy, dtheta):
        self.x += dx
        self.y += dy
        self.theta += dtheta



class BallDetector:
    def __init__(self, calibration, distortion, confidence_threshold, ball_diameter_cm):
        self.calibration = calibration
        self.distortion = distortion
        self.confidence_threshold = confidence_threshold
        self.ball_diameter_cm = ball_diameter_cm
        self.model = YOLO(r'C:\Users\robin\Documents\ECE4191_E34\RobitShit\best.pt')  
        self.camera = cv2.VideoCapture(0)
        self.detected = False

    def detect(self, frame):
        #undistort the frame
        
        frame = cv2.flip(frame, -1) # vertical
        frame = cv2.undistort(frame, self.calibration, self.distortion)

        
        
        #YOLO inference
        results = self.model(frame)

        for result in results:
            boxes = result.boxes
            for box in boxes:
                #Bounding box coords
                xyxy = box.xyxy[0].tolist()
                confidence = box.conf[0].item()

                if confidence >= self.confidence_threshold:
                    x1, y1, x2, y2 = xyxy
                    perceived_diameter_px = max(x2 - x1, y2 - y1)
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    if perceived_diameter_px > 0:
                        x_position = (self.ball_diameter_cm * self.calibration[0, 0]) / perceived_diameter_px
                        y_position = (center_x - self.calibration[0, 2]) * x_position / self.calibration[0, 0]
                        height = -(center_y - self.calibration[1, 2]) * x_position / self.calibration[1, 1]
                        
                        self.detected = True
                        
                        return x_position, y_position, height, confidence
        return None, None,None, None


#
#Simple Planner 
#This planner is a simple planner that moves the robot to a target position
#
#
#move(self, target_x, target_y) 
#Checks if the robot will stay within the boundary
#
#valid_ball_position(self, robot_pose, ball_pose)
#Checks if the ball is within the boundary
#
#get_position(self)
#Returns the current position of the robot
#



class SimplePlanner:
    def __init__(self, boundary_width, boundary_height, start_x, start_y,boundary_threhold):
        self.boundary_width = boundary_width
        self.boundary_height = boundary_height
        self.position_x = start_x
        self.position_y = start_y
        self.boundary_threhold = boundary_threhold

    def move(self, target_x, target_y):
        #make sure robot stays within boundary
        if self.boundary_threhold <= target_x < self.boundary_width - self.boundary_threhold:
            self.position_x = target_x
        if self.boundary_threhold <= target_y < self.boundary_height - self.boundary_threhold:
            self.position_y = target_y

    def valid_ball_position(self, robot_pose, ball_pose):
        x = robot_pose.x + ball_pose[0] * np.cos(robot_pose.theta) - ball_pose[1] * np.sin(robot_pose.theta)
        y = robot_pose.y + ball_pose[0] * np.sin(robot_pose.theta) + ball_pose[1] * np.cos(robot_pose.theta)

        return boundary_threhold <= x < self.boundary_width-boundary_threhold and boundary_threhold <= y < self.boundary_height-boundary_threhold

    def get_position(self):
        return self.position_x, self.position_y



#robot pose
robot_pose = RobotPose(0, 0, 0)

#detector 
calibration_matrix = np.array([[649.47113436, 0.0, 249.59894886],
                               [0.0, 650.00394065, 252.46785209],
                               [0.0, 0.0, 1.0]])

distortion_coeffs = np.array([[-0.12860389, -0.3649167, 0.00291756, -0.00061939, 0.32511248]])
confidence_threshold = 0.7
ball_diameter_cm = (6.54 + 6.86) / 2

ball_detector = BallDetector(calibration_matrix, distortion_coeffs, confidence_threshold, ball_diameter_cm)

#boundary 
boundary_width = 4.1148
boundary_height = 5.4864
boundary_threhold = 0.25
# planner = SimplePlanner(boundary_width, boundary_height, start_x=4.1148, start_y=0, boundary_threhold)



while not ball_detector.detected:
    ret, frame = ball_detector.camera.read()

    #move section 
    
    #stop
    #scan
    
    if not ret:
        print("Failed to grab frame")
        break

    x_position, y_position, height, confidence = ball_detector.detect(frame)

    cv2.imshow("YOLOv8 Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ball_detector.camera.release()
cv2.destroyAllWindows()