import time
from sensing import Sensors
from controller import Controller
from interpret import Interpreter
from picarx_improved import Picarx


def follow_line():
    sensor = Sensors()

    # setup car things
    interpreter = Interpreter()
    car = Picarx()
    controller = Controller(car)

    input("Press enter to start")

    while(True):
        values = sensor.read()
        print(values)
        print([a/b for a,b in zip(values,sensor.read)])
        controller.control(interpreter.processing(values,sensor.read))
        car.forward(30)
        time.sleep(0.1)
        #car.set_dir_servo_angle(-20)


if __name__ == "__main__":
    follow_line()
