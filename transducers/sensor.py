from gpiozero import DistanceSensor

class Ultrasonic(object):
    def __init__(self, echo, trigger):
        self.sensor = DistanceSensor(echo, trigger, queue_len=1)
    
    def read(self):
        return self.sensor.distance
