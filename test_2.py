from transducers.grabber import Claw
from transducers.stepper import Rail
from time import sleep

claw = Claw(pin=18)
rail = Rail(motor_pins=[12, 16, 20, 21], start_pos=0)

rail.set_position(-0.3)

# while True:
#     claw.grab()
#     rail.set_position(0.3)
#     sleep(5)
#     rail.set_position(0)
#     claw.release()