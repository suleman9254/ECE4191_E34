import gpiozero
import time
import math
import atexit

class Motor(object):
    def __init__(self, in1, in2, enable, freq, encA, encB, max_count):
        self.max_count = max_count

        self.in1 = gpiozero.OutputDevice(pin=in1)
        self.in2 = gpiozero.OutputDevice(pin=in2)
        self.pwm = gpiozero.PWMOutputDevice(pin=enable, frequency=freq)
        self.enc = gpiozero.RotaryEncoder(a=encA, b=encB, max_steps=max_count)

        self.direction = None

        # Register the release method to be called upon program exit
        atexit.register(self.release)
    
    def release(self):
        """Release GPIO resources."""
        self.in1.close()
        self.in2.close()
        self.pwm.close()
        self.enc.close()

    def change_direction(self, dir):
        """Change the motor's direction based on the desired direction."""
        if dir:  # True for forward direction
            self.in1.on()
            self.in2.off()
        else:    # False for backward direction
            self.in1.off()
            self.in2.on()
    
    def change_pwm(self, duty, dt=None):
        """
        Change the motor's PWM duty cycle.
        
        Parameters:
        - duty: The desired duty cycle (-1 to 1).
        - dt: Time interval to calculate velocity (optional).
        
        Returns:
        - w: The angular velocity if dt is provided, otherwise 0.
        """
        desired_direction = duty > 0
        if desired_direction != self.direction:
            self.change_direction(desired_direction)
            self.direction = desired_direction

        self.pwm.value = abs(duty)
        
        # Calculate velocity if dt is provided
        w = self.read_velocity(dt) if dt else 0
        return w
    
    def read_velocity(self, dt):
        """
        Read the motor's velocity based on encoder steps over a time interval.
        
        Parameters:
        - dt: Time interval to calculate velocity.
        
        Returns:
        - w: The angular velocity of the motor (rad/s).
        """
        initial_steps = self.enc.steps  # Read initial encoder steps
        time.sleep(dt)  # Wait for dt duration
        steps = self.enc.steps - initial_steps  # Calculate the change in steps

        # Calculate rotations based on encoder steps and max count per rotation
        rots = steps / self.max_count
        w = 2 * math.pi * rots / dt  # Calculate angular velocity in rad/s
        return w
