import numpy as np
from time import sleep

from busio import I2C
from board import SCL, SDA
from adafruit_vl53l0x import VL53L0X

from gpiozero import DistanceSensor
from gpiozero import DigitalOutputDevice as DigOut

class ToF(object):
    def __init__(self):

        i2c = I2C(SCL, SDA)
        self.sensor = VL53L0X(i2c)
    
    def read(self):
        meas = [self.sensor.range / 1000]
        return np.array(meas)
    
class ToFPanel(object):
    def __init__(self, pins):
        
        i2c = I2C(SCL, SDA)
        xshut = [DigOut(p, initial_value=False) for p in pins]
        
        self.sensors = []
        for i, pin in enumerate(xshut):
            
            sleep(0.5); pin.on()
            sensor = VL53L0X(i2c)

            if i < len(xshut) - 1:
                sensor.set_address(i + 0x30)
            
            self.sensors.append(sensor)
    
    def read(self):
        meas = [sensor.range / 1000 for sensor in self.sensors]
        return np.array(meas)

    
class Ultrasonic(object):
    def __init__(self, trig, echo):

        trig = [trig] if not isinstance(trig, list) else trig
        echo = [echo] if not isinstance(echo, list) else echo
        self.sensors = [DistanceSensor(echo=e, trigger=t) for e, t in zip(echo, trig)]
    
    def read(self):
        meas = [sensor.distance for sensor in self.sensors]
        return np.array(meas)



