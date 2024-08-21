import time
from gpiozero import InputDevice

class RotaryEncoder(object):
    def __init__(self, a, b, cpr, g_r):
        self.a = InputDevice(a, pull_up=True)
        self.b = InputDevice(b, pull_up=True)

        self.a.pin.edges = 'both'
        self.b.pin.edges = 'both'

        self.a.pin.when_changed = self._a_changed
        self.b.pin.when_changed = self._b_changed

        self._edge = 0
        self.count = 0
        self.max_count = cpr * g_r
    
    def _a_changed(self, ticks, state):
        edge = (self.a._state_to_value(state) << 1) | (self._edge & 0x1)
        if edge:
            self.count += 1

    def _b_changed(self, ticks, state):
        edge = (self._edge & 0x2) | self.b._state_to_value(state)
        if edge:
            self.count += 1
    
    def speed(self, dt):
        self.count = 0
        time.sleep(dt)
        rots = self.count / self.max_count
        return rots / dt 

