from gpiozero import Servo

class Claw(object):
    def __init__(self, pin):
        self.servo = Servo(pin, initial_value=-1)
    
    def grab(self):
        self.servo.value = 1
    
    def release(self):
        self.servo.value = -1