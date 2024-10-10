import numpy as np
from time import sleep

from transducers.grabber import Claw
from transducers.stepper import Rail
from transducers.sensors import Ultrasonic, ToFPanel

from navigation.motor import Motor
from navigation.controller import PIController
from navigation.kinematics import DiffDriveModel
from navigation.planner import TentaclePlanner

from system.alt3 import Robot

max_count = 48 * 75
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)

encoder_delay = 0.02
dt = 2 * encoder_delay + 0.00894649198
controller = PIController(Kp=0.15, Ki=0.02, wheel_radius=0.028, wheel_sep=0.22)
model = DiffDriveModel(motorL=motorL, motorR=motorR, dt=dt, wheel_radius=0.028, wheel_sep=0.22, encoder_delay=encoder_delay)

plannerSteps = 5*3
v, w = np.arange(-0.2, 0.3, 0.025), np.arange(-0.3, 0.3, 0.1)
planner = TentaclePlanner(v, w, steps=plannerSteps, alpha=1, beta=0.1, dt=dt)

# claw = Claw(pin=18)
# rail = Rail(motor_pins=[12, 16, 20, 21])

# tof = ToFPanel(pins=[7, 8, 25])
# ultrasonic = Ultrasonic(trig=9, echo=11)

# params = {'xBox': 4, 'yBox': -3, 'xBound': 4, 'yBound': 3, 'ballRadius': 0.0325, 'boxHeight': 0.13}

robot = Robot(model, controller, planner)
robot.drive(goal=(1, 0, 0), stallTime=20)