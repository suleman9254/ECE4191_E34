import cv2 as cv
from ultralytics import YOLO

class YOLODetector(object):
    def __init__(self, path=r'/home/g34/my_codebase/archive/RobitShit/best.pt', thresh=0.5):
        self.model = YOLO(path)
        self.thresh = thresh
    
    def find_ball(self, frame):
        
        results = self.model(frame)
        detections, centroid, r_px = [], [], []

        for result in results:
            for box in result.boxes:

                x1, y1, x2, y2 = box.xyxy[0]
                confidence = box.conf[0].item()  

                if confidence >= self.thresh:
                    r_px = max(x2 - x1, y2 - y1) / 2
                    centroid = ((x1 + x2) / 2, (y1 + y2) / 2)

                    detections.append((centroid, r_px))
        
        if detections:
            (centroid, r_px) = max(detections, key=lambda x: x[1])
            self.draw_circle(frame, centroid, r_px)
        
        return frame, centroid, r_px
    
    def draw_circle(self, frame, centroid, radius):
        cv.circle(frame, (int(centroid[0]), int(centroid[1])), int(radius), (0, 255, 0), 4)
        cv.circle(frame, (int(centroid[0]), int(centroid[1])), 2, (0, 0, 255), 3)
        return None