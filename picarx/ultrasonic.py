import time
import statistics
from rossros import Bus

try:
    from robot_hat import Pin
    from ultrasonic import Ultrasonic
except ImportError:
    print("This computer does not appear to be a PiCar-X system (robot_hat is not present). Shadowing hardware calls with substitute functions ")
    from sim_robot_hat import *


class Ultrasonic_Sensor(object):
    def __init__(self):
        # ultrasonic sensors
        self.pin1 = Pin("D2")
        self.pin2 = Pin("D3")
        self.sensor = Ultrasonic(self.pin1, self.pin2)

    def read(self):
        return self.sensor.read()



class Ultrasonic_Interpreter(object):
    def __init__(self, stop_threshold = 10):
        self.stop_threshold = stop_threshold
        self.stop = False

    def interpret_obstacle(self, distance):
        self.stop = True if distance < self.stop_threshold else False
        return self.stop



class Ultrasonic_Controller(object):
    def __init__(self,car,speed=20):
        self.speed = speed
        self.car = car

    def control(self, stop: bool):
        if stop:
            self.car.stop()
        else:
            self.car.forward(self.speed)
