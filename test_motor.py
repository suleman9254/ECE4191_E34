import time
import math
from navigation.motor import Motor
from navigation.controller import PIController
from navigation.kinematics import DiffDriveModel

max_count = 48 * 75
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)

motorR.change_pwm(0.5, dt=2)
motorR.change_pwm(0, dt=2)