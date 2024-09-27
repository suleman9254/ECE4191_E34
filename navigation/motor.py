import gpiozero
import time
import math

class Motor(object):
    def __init__(self, in1, in2, enable, freq, encA, encB, max_count):

        self.in1 = gpiozero.OutputDevice(pin=in1)
        self.in2 = gpiozero.OutputDevice(pin=in2)
        self.pwm = gpiozero.PWMOutputDevice(pin=enable, frequency=freq)
        self.enc = gpiozero.RotaryEncoder(a=encA, b=encB, max_steps=0)

        self.set_dir(True)
        self.off, self.max_count = 0.05, max_count

    def set_dir(self, dir):

        if dir == True:
            self.in1.on()
            self.in2.off()
        
        elif dir == False:
            self.in1.off()
            self.in2.on()
        
        else:
            self.in1.off()
            self.in2.off()
        
        self.dir = dir
    
    def set_pwm(self, duty):
        
        dir = duty > 0
        if abs(duty) < self.off:
            dir = None
        
        if dir != self.dir:
            self.set_dir(dir)

        self.pwm.value = abs(duty)
    
    def read_velocity(self, dt):
        self.enc.steps = 0
        time.sleep(dt)
        rots = 4 * self.enc.steps / self.max_count
        w = 2 * math.pi * rots / dt
        return w




        

        
