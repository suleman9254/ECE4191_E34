import gpiozero
import time
import math
import atexit

class Motor(object):
    def __init__(self, in1, in2, enable, freq, encA, encB, max_count):
        self.max_count = max_count

        self.in1 = gpiozero.OutputDevice(pin=in1)
        self.in2 = gpiozero.OutputDevice(pin=in2)
        self.pwm = gpiozero.PWMOutputDevice(pin=enable, frequency=freq)
        self.enc = gpiozero.RotaryEncoder(a=encA, b=encB, max_steps=0)

        self.change_direction(True)
        atexit.register(self.release)
    
    def release(self):
        self.in1.close()
        self.in2.close()
        self.pwm.close()
        self.enc.close()

    def change_direction(self, dir):
        self.dir = dir

        if self.dir == True:
            self.in1.on()
            self.in2.off()
        
        elif self.dir == False:
            self.in1.off()
            self.in2.on()
        
        else:
            self.in1.off()
            self.in2.off()
    
    def change_pwm(self, duty, dt=None):
        desired_direction = duty > 0
        if desired_direction != self.dir:
            self.change_direction(desired_direction)

        self.pwm.value = abs(duty)
        w = 0 if dt is None else self.read_velocity(dt)
        return w
    
    def read_velocity(self, dt):
        self.enc.steps = 0
        time.sleep(dt)
        rots = 4 * self.enc.steps / self.max_count
        w = 2 * math.pi * rots / dt
        return w




        

        
