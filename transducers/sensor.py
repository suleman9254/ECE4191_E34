from busio import I2C
from board import SCL, SDA
from adafruit_vl53l0x import VL53L0X as Sensor

class DistanceSensor(object):
    def __init__(self):

        i2c = I2C(SCL, SDA)
        self.sensor = Sensor(i2c)
    
    def read(self):
        return self.sensor.range / 1000
