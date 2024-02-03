import time
from concun_sense import Sensors
from controller import Controller
from concun_interpret import Interpreter
from picarx_improved import Picarx
import concurrent.futures


def follow_line():
    sensor = Sensors()
    input("Press enter to calibrate grayscale, make sure all sensors are on white")

    sensor.calibrate_grayscale()

    # setup car things
    interpreter = Interpreter()
    car = Picarx()
    controller = Controller(car)

    input("Press enter to start")

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        eSensor = executor.submit(sensor.produce, sensor.bus, sensor.delay)
        eInterpreter = executor.submit(interpreter.produce_consume, sensor.bus, interpreter.bus, interpreter.delay)
        eController = executor.submit(controller.consume, interpreter.bus, controller.delay)

    eSensor.result()
    eInterpreter.result()
    eController.result()

    '''
    while(True):
        values = sensor.read()
        print(values)
        print([a/b for a,b in zip(values,sensor.grayscale_cal_values)])
        controller.control(interpreter.processing(values,sensor.grayscale_cal_values))
        car.forward(20)
        time.sleep(0.1)
    '''
if __name__ == "__main__":
    follow_line()