import time
from bus import Bus

class Controller(object):
    def __init__(self,car,scale=30,delay=0.05):
        self.scale = scale
        self.car = car
        self.running = False
        self.bus = Bus()
        self.delay = delay

    def control(self, offset):
        steering_angle = offset * self.scale
        self.car.set_dir_servo_angle(steering_angle)

    def consume(self, interpreter_bus, delay):
        self.running = True
        while self.running:
            self.control(interpreter_bus.read())
            time.sleep(delay)
