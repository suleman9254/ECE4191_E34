from time import sleep
from gpiozero import Servo

class Claw(object):
    def __init__(self, pin, delay=0.3, min_pulse_width=0.0004, max_pulse_width=0.0024, frame_width=0.02):
        
        self.delay = delay
        self.servo = Servo(pin=pin, 
                           initial_value=None, 
                           min_pulse_width=min_pulse_width, 
                           max_pulse_width=max_pulse_width, 
                           frame_width=frame_width)
    
    def grab(self):
        self.servo.min()
        sleep(self.delay)
        self.servo.detach()
    
    def release(self):
        self.servo.max()
        sleep(self.delay)
        self.servo.detach()