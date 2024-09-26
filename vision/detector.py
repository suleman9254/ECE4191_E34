import cv2 as cv
import numpy as np
from ultralytics import YOLO

class YOLODetector(object):
    def __init__(self, box_ckpt, ball_ckpt, thresh):
        
        self.thresh = thresh
        self.boxNet = YOLO(box_ckpt)
        self.ballNet = YOLO(ball_ckpt)
    
    def detect(self, frame, model):
        output, confs = [], []
        results = model(frame)
        
        for result in results:
            for box in result.boxes:
                
                confidence = box.conf[0].item()
                if confidence >= self.thresh:
                    confs.append( confidence )
                    output.append( box.xyxy[0] )
        
        return output, confs

    def find_ball(self, frame):
        centroid, radius = None, None
        results, _ = self.detect(frame, self.ballNet)
        
        if len(results):
            box = max(results, key=lambda x: self._radius(x))
        
            radius = self._radius(box)
            centroid = self._centroid(box)
        
        return frame, centroid, radius

    def find_box(self, frame):
        centroid, height = None, None
        results, confidence = self.detect(frame, self.boxNet)
        
        if len(results):
            box = results[ np.argmax(confidence) ]
            
            centroid = self._centroid(box)
            height = self._height(box)
        
        return frame, centroid, height
    
    def draw_circle(self, frame, centroid, radius):
        cv.circle(frame, centroid, radius, (0, 255, 0), 4)
        cv.circle(frame, centroid, 2, (0, 0, 255), 3)
    
    def _radius(self, box):
        x1, y1, x2, y2 = box
        diameter = max(x2 - x1, y2 - y1)
        return int( diameter / 2 )

    def _centroid(self, box):
        x1, y1, x2, y2 = box
        x, y = (x1 + x2) / 2, (y1 + y2) / 2
        return ( int(x) , int(y) )

    def _height(self, box):
        _, y1, _, y2 = box
        return int( y2 - y1 )