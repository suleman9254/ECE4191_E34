from navigation.motor import Motor

max_count = 48 * 75
motor_l = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count, dt=0.1)

w = motor_l.change_pwm(0.7) 
print(w)
