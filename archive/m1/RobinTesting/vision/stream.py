import cv2 as cv
import numpy as np
from flask import Flask, Response

class Stream(object):
    def __init__(self, host='0.0.0.0', port=5000):        
        self.app = Flask(__name__)
        self.app.add_url_rule('/', 'video_feed', self.video_feed)

        self.host, self.port = host, port
        self.frame = np.zeros((720, 1280, 3))
    
    def set_frame(self, frame):
        self.frame = frame

    def get_frame(self):
        _, buffer = cv.imencode('.jpg', self.frame)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    
    def video_feed(self):
        return Response(self.get_frame(), mimetype='multipart/x-mixed-replace; boundary=frame')
    
    def start(self):
        self.app.run(host=self.host, port=self.port)