from navigation.motor import Motor
import time

motor_l = Motor(in1=17, in2=27, enable=22, freq=100, encA=5, encB=6, steps=48*75, dt=0.1)

w = motor_l.change_pwm(-0.5)
print(w)

time.sleep(10)