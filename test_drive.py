from navigation.kinematics import DiffDriveModel
from navigation.motor import Motor
from navigation.controller import PIController

motor_l = Motor(in1=4, in2=17, enable=27, freq=100, encA=22, encB=5, steps=48, dt=0.1)
motor_r = Motor(in1=18, in2=23, enable=24, freq=100, encA=25, encB=12, steps=48, dt=0.1)

my_car = DiffDriveModel(motor_l, motor_r, dt=0.1, wheel_radius=0.028, wheel_sep=0.15)
controller = PIController(Kp=0.1, Ki=0.01, wheel_radius=0.028, wheel_sep=0.15)

for i in range(300):

    if i < 100: # drive in circular path (turn left) for 10 s
        duty_cycle_l, duty_cycle_r = controller.drive(0.1, 1, my_car.wl, my_car.wr)
    elif i < 200: # drive in circular path (turn right) for 10 s
        duty_cycle_l, duty_cycle_r = controller.drive(0.1, -1, my_car.wl, my_car.wr)
    else: # stop
        duty_cycle_l, duty_cycle_r = (0,0)
    
    # Simulate robot motion - send duty cycle command to robot
    x,y,th = my_car.pose_update(duty_cycle_l,duty_cycle_r)
    
    # Log data
    print(my_car.wl)
    print(my_car.wr)

    print('\n')
    