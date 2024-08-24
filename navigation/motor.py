import gpiozero
import time
import math

class Motor(object):
    def __init__(self, in1, in2, enable, freq, encA, encB, max_count, dt):
        self.dt = dt
        self.max_count = max_count

        self.in1 = gpiozero.OutputDevice(pin=in1)
        self.in2 = gpiozero.OutputDevice(pin=in2)
        self.pwm = gpiozero.PWMOutputDevice(pin=enable, frequency=freq)
        self.enc = gpiozero.RotaryEncoder(a=encA, b=encB, max_steps=0)
    
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
        return self.read_velocity(self.dt)    
    
    def read_velocity(self, dt):
        self.enc.steps = 0
        time.sleep(dt)
        rots = 4 * self.enc.steps / self.max_count
        w = 2 * math.pi * rots / dt
        return w




        

        
