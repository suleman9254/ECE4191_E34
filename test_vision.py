from vision.camera import Camera
from vision.stream import Stream
from vision.detection import Detector, DetectorConfig

camera = Camera(cam_idx=0)

cfg = DetectorConfig(hsv_low=[20, 100, 100], hsv_high=[40, 255, 255])
detector = Detector(cfg)

stream = Stream(camera, detector, port=5000)
stream.start()