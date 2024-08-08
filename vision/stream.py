import cv2 as cv
import numpy as np
from flask import Flask, Response

class Stream(object):
    def __init__(self, camera, detector, host='0.0.0.0', port=5000):        
        
        self.host = host
        self.port = port
        self.camera = camera
        self.detector = detector

        self.app = Flask(__name__)
        self.app.add_url_rule('/', 'video_feed', self.video_feed)
    
    def get_frame(self):
        while True:
            frame = self.camera.read_frame()
            circle = self.detector.find_ball(frame)
            
            if circle is not None:
                circle = np.round(circle).astype("int")
                (x, y, r) = circle

                cv.circle(frame, (x, y), r, (0, 255, 0), 4)
                cv.circle(frame, (x, y), 2, (0, 0, 255), 3)
            
            _, buffer = cv.imencode('.jpg', frame)
            frame = buffer.tobytes()

            yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    
    def video_feed(self):
        return Response(self.get_frame(), mimetype='multipart/x-mixed-replace; boundary=frame')
    
    def start(self):
        self.app.run(host=self.host, port=self.port)