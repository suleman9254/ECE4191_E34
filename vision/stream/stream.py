import cv2 as cv
from flask import Flask, Response, render_template, redirect, url_for
from time import time

class Stream(object):
    def __init__(self, camera, host='0.0.0.0', port=5000):        
        self.host = host
        self.port = port
        self.camera = camera

        self.app = Flask(__name__)
        self.app.add_url_rule('/', 'index', self.index)
        self.app.add_url_rule('/video_feed', 'video_feed', self.video_feed)
        self.app.add_url_rule('/capture', 'capture', self.capture, methods=['POST'])

    def index(self):
        return render_template('index.html')

    def get_frame(self):
        while True:
            frame = self.camera.read_frame()
            _, buffer = cv.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def video_feed(self):
        return Response(self.get_frame(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def capture(self):
        frame = self.camera.read_frame()
        pth = f"vision/pics/{time()}.jpg"
        cv.imwrite(pth, frame)
        return redirect(url_for('index'))

    def start(self):
        self.app.run(host=self.host, port=self.port)
