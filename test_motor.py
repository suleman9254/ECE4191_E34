from navigation.motor import Motor
import math
import time

motor_l = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, steps=48, dt=0.5)
w = motor_l.change_pwm(0.3) 

wait = 5

time.sleep(wait)
rots = w * wait

print(rots)

