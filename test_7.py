from time import sleep
from transducers.sensor import DistanceSensor

sensor = DistanceSensor()

while True:
    print(sensor.read())
    sleep(0.5)
