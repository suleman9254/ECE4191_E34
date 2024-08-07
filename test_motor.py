from motor import Motor
import time

motor_l = Motor(in1=4, in2=17, enable=27, freq=100, encA=22, encB=5, steps=48, dt=0.1)

motor_l.change_pwm(0.5)

time.sleep(10)