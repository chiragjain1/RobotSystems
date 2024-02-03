import time
import statistics
from rossros import Bus

try:
    from robot-hat import ADC
except ImportError:
    print("This computer does not appear to be a PiCar-X system (robot_hat is not present). Shadowing hardware calls with substitute functions ")
    from sim_robot_hat import *

class Grayscale_Sensor(object):
    def __init__(self):
        # grayscale sensors
        self.chn0 = ADC('A0')
        self.chn1 = ADC('A1')
        self.chn2 = ADC('A2')
        self.grayscale_cal_values = []

    def read(self):
        adc_value_list = []
        adc_value_list.append(self.chn0.read())
        adc_value_list.append(self.chn1.read())
        adc_value_list.append(self.chn2.read())
        return adc_value_list

    def calibrate_grayscale(self):
        x = 0
        ch0 = []
        ch1 = []
        ch2 = []
        while (x < 10):
            time.sleep(0.1)
            values = self.read()
            ch0.append(values[0])
            ch1.append(values[1])
            ch2.append(values[2])
            x = x + 1
        self.grayscale_cal_values.append(statistics.mean(ch0))
        self.grayscale_cal_values.append(statistics.mean(ch1))
        self.grayscale_cal_values.append(statistics.mean(ch2))

    '''
    def produce(self, sensor_bus: Bus, delay):
        while True:
            message = self.read()
            sensor_bus.set_message(message)
            time.sleep(delay)
    '''

class Grayscale_Interpreter(object):
    def __init__(self,sensitivity=0.1,polarity=1):
        self.sensitivity = sensitivity * polarity
        self.running = False

    def processing(self, values, cal_values):
        values = [x+1 if x == 0 else x for x in values]
        left, middle, right = values

        left, middle, right = values
        left_cal, middle_cal, right_cal = cal_values
        left = left/left_cal
        middle = middle/middle_cal
        right = right/right_cal

        if abs(right - left) > self.sensitivity: # means we're off the line
            if right > left:
                print("left")
                turn = (left - middle) / (middle+left)
            else: # left > right:
                turn = (middle - right) / (middle+right)
        else: # we're perfect
            turn = 0

        return turn

    '''
    def produce_consume(self, sensor_bus: Bus, interpreter_bus: Bus, delay):
        self.running = True
        while self.running:
            message = sensor_bus.get_message()
            value = self.processing(message)
            interpreter_bus.set_message(value)
            time.sleep(delay)
    '''

class Grayscale_Controller(object):
    def __init__(self,car,scale=30,delay=0.05):
        self.scale = scale
        self.car = car
        self.running = False
        self.delay = delay

    def control(self, offset):
        steering_angle = offset * self.scale
        self.car.set_dir_servo_angle(steering_angle)
        self.car.forward(30)

    '''
    def consume(self, interpreter_bus: Bus, delay):
        self.running = True
        while self.running:
            message = interpreter_bus.get_message()
            self.control(message)
            time.sleep(delay)
    '''



if __name__ == "__main__":
    sensor = Grayscale_Sensor()
    x = 0
    while (x < 1000000):
        time.sleep(0.5)
        print(sensor.read())
        x = x + 0.1