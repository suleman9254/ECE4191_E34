import time
from navigation.motor import Motor

max_count = 48 * 75
# motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=20, encB=21, max_count=max_count)

wL = motorL.change_pwm(0.5, dt=0.1)
# wR = motorR.change_pwm(0.5, dt=0.1)

# print(wR)
print(wL)

time.sleep(5)
