from picarx_improved import Picarx
import atexit

class Controller(object):
    def __init__(self,car,scale=50):
        self.scale = scale
        self.car = car

    def control(self, offset):
        steering_angle = offset * self.scale
        self.car.set_dir_servo_angle(steering_angle)

