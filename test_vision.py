from vision.camera import Camera
from vision.stream import Stream
from vision.detection import Detector, DetectorConfig

camera = Camera(cam_idx=0)

cfg = DetectorConfig(gauss_k=5, 
                     gauss_sig=0, 
                     cht_p1=50, 
                     cht_p2=55, 
                     cht_dp=0.5, 
                     cht_min_d=40, 
                     hsv_low=[35, 61, 44],
                     hsv_high=[53, 201, 222])
detector = Detector(cfg)

stream = Stream(camera, detector, port=5000)
stream.start()