from time import sleep
from gpiozero import Servo

class Claw(object):
    def __init__(self, pin, off_delay=0.3, exit_delay=1, min_pulse_width=0.0014, max_pulse_width=0.0022, frame_width=0.02):
        
        self.servo = Servo(pin=pin, 
                           initial_value=None, 
                           min_pulse_width=min_pulse_width, 
                           max_pulse_width=max_pulse_width, 
                           frame_width=frame_width)

        self.off_delay = off_delay
        self.exit_delay = exit_delay
    
    def grab(self):
        self.servo.min()
        sleep(self.off_delay)
        self.servo.detach()
        sleep(self.exit_delay)
    
    def release(self):
        self.servo.max()
        sleep(self.off_delay)
        self.servo.detach()
        sleep(self.exit_delay)