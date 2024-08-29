from ultralytics import YOLO
import cv2 as cv
import numpy as np

class YOLODetector(object):
    def __init__(self, path, thresh):
        """
        Initializes the YOLO object detection model.
        
        Parameters:
        - path: Path to the YOLO model file.
        - thresh: Confidence threshold for detections.
        """
        self.model = YOLO(path)
        self.thresh = thresh
        self.actual_diameter_cm = ((6.54 + 6.86) / 2)  /100
        self.camera_matrix = np.load('vision/params/camera_matrix.npy')
        

    
    def find_ball(self, frame):
        """
        Detects balls in a given frame using YOLO model.
        
        Parameters:
        - frame: The input image frame (numpy array).
        
        Returns:
        - frame: The input frame with detected balls highlighted.
        - center: Tuple (center_x, center_y) of the detected ball.
        - r_px: The radius of the detected ball in pixels.
        """
        center_x = None
        center_y = None
        r_px = None

        # Perform object detection
        results = self.model(frame)

        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Extract bounding box coordinates
                xyxy = list(box.xyxy[0])
                
                # Confidence score
                confidence = box.conf[0].item()

                # Check if confidence meets threshold
                if confidence >= self.thresh:
                    x1, y1, x2, y2 = xyxy
                    r_px = max(x2 - x1, y2 - y1) / 2
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    # Draw the bounding box and center on the frame
                    self.draw_detection(frame, (center_x, center_y), r_px)
                    perceived_diameter_px = max(x2 - x1, y2 - y1)
                    if perceived_diameter_px > 0:
                        x_position = (self.actual_diameter_cm * self.camera_matrix[0, 0]) / perceived_diameter_px
                        print(f"Estimated X: {x_position:.2f} cm")
                       
                        
                        y_position = (center_x - self.camera_matrix[0, 2]) * x_position / self.camera_matrix[0, 0]
                        print(f"Estimated Y: {y_position:.2f} cm")
                         
                         
                        height  = -(center_y - self.camera_matrix[1, 2]) * x_position / self.camera_matrix[1, 1]        
                    
        
        

        return frame, (center_x, center_y), r_px
    
    
    def draw_detection(self, frame, center, radius):
        """
        Draws the detection results on the frame.
        
        Parameters:
        - frame: The input image frame.
        - center: Tuple (center_x, center_y) of the detected ball.
        - radius: The radius of the detected ball in pixels.
        """
        if center[0] is not None and center[1] is not None:
            cv.circle(frame, (int(center[0]), int(center[1])), int(radius), (0, 255, 0), 2)
            cv.circle(frame, (int(center[0]), int(center[1])), 2, (0, 0, 255), 3)

        return None
