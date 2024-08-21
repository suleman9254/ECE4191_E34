import gpiozero
import time
import math

class Motor(object):
    def __init__(self, in1, in2, enable, freq):
        self.in1 = gpiozero.OutputDevice(pin=in1)
        self.in2 = gpiozero.OutputDevice(pin=in2)
        self.pwm = gpiozero.PWMOutputDevice(pin=enable, frequency=freq)
    
    def change_direction(self, dir):
        if dir == 'off':
            self.in1.off()
            self.in2.off()

        elif dir == 'clockwise':
            self.in1.on()
            self.in2.off()
        
        elif dir == 'anticlockwise':
            self.in1.off()
            self.in2.on()
    
    def change_pwm(self, duty):
        if duty >= 0:
            self.change_direction('clockwise') 
        else: 
            self.change_direction('anticlockwise')

        self.pwm.value = abs(duty)
        return None

        

        
