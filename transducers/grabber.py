from time import sleep
from gpiozero import Servo

class Claw(object):
    def __init__(self, claw, delay=0.3, initial_value=1, min_pulse_width=0.0004, max_pulse_width=0.0024):
        
        self.delay = delay
        self.servo = Servo(claw, initial_value, min_pulse_width, max_pulse_width)
    
    def grab(self):
        self.servo.min()
        sleep(self.delay)
    
    def release(self):
        self.servo.max()
        sleep(self.delay)