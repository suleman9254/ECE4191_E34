class PIController(object):
    def __init__(self, Kp=0.1, Ki=0.01, wheel_radius=0.02, wheel_sep=0.1):
        """
        Initializes the PI controller for differential drive robot.

        Parameters:
        - Kp: Proportional gain.
        - Ki: Integral gain.
        - wheel_radius: Radius of the wheel.
        - wheel_sep: Separation between the wheels.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.r = wheel_radius
        self.l = wheel_sep
        self.e_sum_l = 0
        self.e_sum_r = 0

    def p_control(self, w_desired, w_measured, e_sum):
        """
        Computes the PI control output for a given wheel.

        Parameters:
        - w_desired: Desired angular velocity.
        - w_measured: Measured angular velocity.
        - e_sum: Accumulated integral error.

        Returns:
        - duty_cycle: Computed duty cycle for the motor.
        - e_sum: Updated accumulated integral error.
        """
        # Compute the error between desired and measured velocity
        error = w_desired - w_measured
        
        # Update integral term
        e_sum += error
        
        # Compute the control output with clamping
        control_output = self.Kp * error + self.Ki * e_sum
        
        # Clamp the duty cycle to [-1, 1] and prevent integral windup
        if control_output > 1:
            duty_cycle = 1
            e_sum -= error  # Anti-windup: Prevent further accumulation of error
        elif control_output < -1:
            duty_cycle = -1
            e_sum -= error  # Anti-windup: Prevent further accumulation of error
        else:
            duty_cycle = control_output

        return duty_cycle, e_sum

    def drive(self, v_desired, w_desired, wl, wr):
        """
        Computes the duty cycles for left and right motors to achieve desired velocities.

        Parameters:
        - v_desired: Desired linear velocity.
        - w_desired: Desired angular velocity.
        - wl: Measured left wheel angular velocity.
        - wr: Measured right wheel angular velocity.

        Returns:
        - duty_cycle_l: Duty cycle for the left motor.
        - duty_cycle_r: Duty cycle for the right motor.
        """
        # Calculate desired angular velocities for each wheel
        wl_desired = (v_desired - (self.l / 2) * w_desired) / self.r
        wr_desired = (v_desired + (self.l / 2) * w_desired) / self.r
        
        # Compute duty cycles using PI control for each wheel
        duty_cycle_l, self.e_sum_l = self.p_control(wl_desired, wl, self.e_sum_l)
        duty_cycle_r, self.e_sum_r = self.p_control(wr_desired, wr, self.e_sum_r)
        
        return duty_cycle_l, duty_cycle_r
