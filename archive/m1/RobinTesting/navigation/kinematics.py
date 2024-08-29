import time
import math
class DiffDriveModel(object):
    def __init__(self, motor_l, motor_r, dt=0.1, wheel_radius=0.05, wheel_sep=0.15, steps_per_revolution=400):
        """
        Initializes the differential drive model.
        
        Parameters:
        - motor_l: Left motor object.
        - motor_r: Right motor object.
        - dt: Time step for each update (s).
        - wheel_radius: Radius of the wheels (m).
        - wheel_sep: Separation between the wheels (m).
        - steps_per_revolution: Encoder steps per wheel revolution.
        """
        self.x = 0.0  # x-position
        self.y = 0.0  # y-position 
        self.th = 0.0  # orientation
        
        self.wl = 0.0  # rotational velocity left wheel
        self.wr = 0.0  # rotational velocity right wheel
        
        self.r = wheel_radius
        self.l = wheel_sep
        self.dt = dt
        self.steps_per_revolution = steps_per_revolution

        self.right_motor = motor_r
        self.left_motor = motor_l

        # Accumulate total steps for each wheel to track total distance
        self.total_steps_left = 0
        self.total_steps_right = 0
    
    def base_velocity(self, dl, dr):
        """
        Compute the base velocity and angular velocity.
        
        Parameters:
        - dl: Distance traveled by the left wheel (m).
        - dr: Distance traveled by the right wheel (m).
        
        Returns:
        - v: Linear velocity (m/s).
        - w: Angular velocity (rad/s).
        """
        v = (dl + dr) / 2.0
        w = (dr - dl) / self.l

        return v, w
    
    def pose_update(self, duty_cycle_l, duty_cycle_r):
        """
        Update the robot's pose based on encoder data and motor inputs.
        
        Parameters:
        - duty_cycle_l: Duty cycle for the left motor.
        - duty_cycle_r: Duty cycle for the right motor.
        
        Returns:
        - Updated pose (x, y, th) and wheel velocities (wl, wr).
        """
        # Update motor speeds based on duty cycles
        self.left_motor.change_pwm(duty_cycle_l)
        self.right_motor.change_pwm(duty_cycle_r)
        
        # Update the accumulated encoder steps for each wheel
        self.total_steps_left += self.left_motor.enc.steps
        self.total_steps_right += self.right_motor.enc.steps

        # Calculate the total distance traveled by each wheel
        dl = (self.total_steps_left / self.steps_per_revolution) * (2 * math.pi * self.r)
        dr = (self.total_steps_right / self.steps_per_revolution) * (2 * math.pi * self.r)
        
        # Calculate velocities
        self.wl = dl / self.dt
        self.wr = dr / self.dt
        
        # Compute the robot's linear and angular velocity
        v, w = self.base_velocity(dl, dr)
        
        # Update the robot's pose
        self.x += self.dt * v * math.cos(self.th)
        self.y += self.dt * v * math.sin(self.th)
        self.th += w * self.dt

        return self.x, self.y, self.th, self.wl, self.wr
    
    def reset_to_initial_position(self):
        """
        Compute the wheel movements required to return to the initial position (0,0).
        This method controls the robot to return to the origin using encoder data.
        """
        # Calculate the Euclidean distance to the origin
        distance_to_origin = math.sqrt(self.x**2 + self.y**2)
        
        # Calculate the angle to the origin from the current orientation
        angle_to_origin = math.atan2(-self.y, -self.x)  # Angle required to face the origin
        
        # Calculate the difference in orientation needed to face the origin
        angle_difference = angle_to_origin - self.th
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))  # Normalize to [-π, π]
        
        # Step 1: Rotate to face the origin
        while abs(angle_difference) > 0.01:  # Small threshold for angle accuracy
            rotation_speed = 0.2 if angle_difference > 0 else -0.2
            duty_cycle_l, duty_cycle_r = self.controller.drive(0, rotation_speed, self.wl, self.wr)
            self.pose_update(duty_cycle_l, duty_cycle_r)
            
            # Update the angle difference after rotating
            angle_to_origin = math.atan2(-self.y, -self.x)
            angle_difference = angle_to_origin - self.th
            angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))
            
            time.sleep(self.dt)
        
        # Step 2: Move forward to the origin
        while distance_to_origin > 0.01:  # Small threshold for distance accuracy
            duty_cycle_l, duty_cycle_r = self.controller.drive(0.1, 0, self.wl, self.wr)  # Move forward
            self.pose_update(duty_cycle_l, duty_cycle_r)
            
            # Update the distance to origin after moving
            distance_to_origin = math.sqrt(self.x**2 + self.y**2)
            
            time.sleep(self.dt)
        
        # Step 3: Rotate to align back to the initial orientation (theta = 0)
        while abs(self.th) > 0.01:  # Small threshold for angle accuracy
            rotation_speed = 0.2 if self.th < 0 else -0.2
            duty_cycle_l, duty_cycle_r = self.controller.drive(0, rotation_speed, self.wl, self.wr)
            self.pose_update(duty_cycle_l, duty_cycle_r)
            
            time.sleep(self.dt)

        print("Robot returned to initial position (0,0) with orientation 0.")
