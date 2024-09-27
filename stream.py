from vision.camera import Camera
from vision.stream.stream import Stream

camera = Camera(0)
stream = Stream(camera)
stream.start()