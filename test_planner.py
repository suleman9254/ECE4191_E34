from navigation.motor import Motor
from navigation.controller import PIController
from navigation.kinematics import DiffDriveModel
from navigation.planner import TentaclePlanner

max_count = 48 * 75
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)

controller = PIController(Kp=0.1, Ki=0.01, wheel_radius=0.028, wheel_sep=0.22)
model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt=0.1, wheel_radius=0.028, wheel_sep=0.22)

planner = TentaclePlanner(dt=0.1, steps=5, alpha=1, beta=0.1)

goal_x = 1
goal_y = 0
goal_th = 0

tolerance = 0.1

while abs(model.x - goal_x) > tolerance or \
        abs(model.y - goal_y) > tolerance:

    v, w = planner.plan(goal_x, goal_y, goal_th, model.x, model.y, model.th)
    duty_cycle_l, duty_cycle_r = controller.drive(v, w, model.wl, model.wr)
    model.pose_update(duty_cycle_l, duty_cycle_r)

    print(abs(model.x - goal_x))


model.pose_update(duty_cycle_l=0, duty_cycle_r=0)