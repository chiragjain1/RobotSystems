from c_line_follow import GrayscaleSensing, Controller, Interpretation
from picarx_improved import Picarx
from rossros import *


def follow_line():
    ref =  [405, 481, 433]
    gs_sensor = GrayscaleSensing("A0", "A1", "A2",ref)
    #input("Press enter to calibrate grayscale, make sure all sensors are on white")
    #gs_sensor.calibrate_grayscale()

    # setup car things
    car = Picarx()
    gs_interpreter = Interpretation(polarity=1)
    gs_controller = Controller(1.0,30)
    #us_sensor = Ultrasonic_Sensor()
    #us_interpreter = Ultrasonic_Interpreter()
    #us_controller = Ultrasonic_Controller(car)
    # setup busses
    gs_interpreter_bus = Bus(initial_message=[0, 0, 0],name="gs_interpreter_bus")
    gs_sensor_bus = Bus(initial_message=0,name="gs_sensor_bus")
    #us_sensor_bus = Bus(initial_message=0,
    #                       name="us_sensor_bus")
    #us_interpreter_bus = Bus(initial_message=False,
    #                            name="us_interpreter_bus")
    delay = 0.05

    input("Press enter to start")

    # grayscale sensor threads
    gs_read = Producer(gs_sensor.read, output_buses=gs_sensor_bus, delay=0.1, name="gs_read")

    raw = gs_sensor.read()
    gs_process = ConsumerProducer(gs_interpreter.calcLineState(raw),
                                      input_buses=gs_sensor_bus,
                                      output_buses=gs_interpreter_bus,
                                      delay=0.1,
                                      name="gs_process")
    gs_control = Consumer(gs_controller.getSteeringAngle(), input_buses=gs_interpreter_bus, delay=0.1, name="gs_control")

    #us_read = Producer(us_sensor.read, output_busses=us_sensor_bus, delay=0.1, name="us_read")

    #us_process = ConsumerProducer(us_interpreter.interpret_obstacle,
    #                                  input_busses=us_sensor_bus,
    #                                  output_busses=us_interpreter_bus,
    #                                  delay=0.1,
    #                                  name="us_process")
    #us_control = Consumer(us_controller.ultra_controller, input_busses=us_interpreter_bus, delay=0.1, name="us_control")

    thread_list = [gs_read, gs_process, gs_control]
    runConcurrently(thread_list)


if __name__ == "__main__":
    follow_line()