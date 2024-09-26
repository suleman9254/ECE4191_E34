from PiicoDev_VL53L1X import PiicoDev_VL53L1X as Sensor

class DistanceSensor(object):
    def __init__(self):
        self.sensor = Sensor()
    
    def read(self):
        return self.sensor.read() / 1000
