from gpiozero import OutputDevice
from time import sleep

class Rail(object):
    
    def __init__(self, motor_pins, start_pos=0, len_per_step=0.04, total_len=165, number_of_steps=200, step_sequence=[[0,1,1,0], [0,1,0,1], [1,0,0,1], [1,0,1,0]]):
        
        self.motor_pins = [OutputDevice(pin) for pin in motor_pins]
        
        self.step_sequence = step_sequence
        self.number_of_steps = number_of_steps

        self.max_steps = total_len / len_per_step
        self.step_number = -start_pos * self.max_steps
        
        self.clockwise = True
        self.set_speed(rpm=240)
        
    def set_speed(self, rpm):
        self.step_delay = 60 / self.number_of_steps / rpm

    def disable(self):
        [pin.off() for pin in self.motor_pins]

    def set_position(self, perc):
        difference = perc * self.max_steps + self.step_number
        clockwise = True if difference > 0 else False

        steps = int( abs(difference) )
        for _ in range(steps):
            self.step_motor(clockwise)

        self.disable()

    def step_motor(self, clockwise):
        self.step_number += -1 if clockwise else 1

        seq_idx = self.step_number % self.number_of_steps
        seq_idx = int(seq_idx % len(self.step_sequence))
        seq = self.step_sequence[seq_idx]
        
        for idx, pin in enumerate(seq):
            
            if pin == 1:
                self.motor_pins[idx].on() 
            else:
                self.motor_pins[idx].off()

        sleep(self.step_delay)