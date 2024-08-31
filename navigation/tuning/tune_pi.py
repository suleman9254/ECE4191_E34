import optuna
import numpy as np
from navigation.motor import Motor
from navigation.kinematics import DiffDriveModel
from navigation.controller import PIController

max_count = 48 * 75
motorL = Motor(in1=5, in2=6, enable=13, freq=100, encA=23, encB=24, max_count=max_count)
motorR = Motor(in1=17, in2=27, enable=22, freq=100, encA=19, encB=26, max_count=max_count)

encoder_delay = 0.02
dt = 2 * encoder_delay + 0.005
model = DiffDriveModel(motor_l=motorL, motor_r=motorR, dt=dt, wheel_radius=0.028, wheel_sep=0.22, encoder_delay=encoder_delay)

ramp_up_time = 1  # 1 second
hold_time = 2  # 2 seconds
ramp_down_time = 2  # 2 seconds

ramp_down = np.linspace(0.3, 0, int(ramp_down_time / dt))
ramp_up = np.linspace(0, 0.3, int(ramp_up_time / dt))
hold = np.full(int(hold_time / dt), 0.3)
vref = np.concatenate([ramp_up, hold, ramp_down])

def run_trial(controller):
    ssd = 0
    
    for v in vref:
        duty_cycle_l, duty_cycle_r = controller.drive(v=v, w=0, wl=model.wl, wr=model.wr)
        model.pose_update(duty_cycle_l, duty_cycle_r)
        ssd = ssd + (v - model.v)**2
    
    model.pose_update(0, 0)
    return ssd

def objective(trial):
    Kp = trial.suggest_float('Kp', 0.01, 0.1, step=0.0045)
    Ki = trial.suggest_float('Ki', 0.001, 0.01, step=0.0009)
    
    controller = PIController(Kp, Ki, wheel_radius=0.028, wheel_sep=0.22)
    ssd = run_trial(controller)
    return ssd

def callback(study, trial):
    if study.best_trial == trial:
        print(study.best_params)

study = optuna.create_study(storage="sqlite:///tune_pi.sqlite3", study_name='PI Tuning')
study.optimize(objective, n_trials=50)