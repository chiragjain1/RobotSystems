import os
import logging
from time import sleep
import time

class _Basic_class(object):
    _class_name = '_Basic_class'
    DEBUG_LEVELS = {'debug': logging.DEBUG,
              'info': logging.INFO,
              'warning': logging.WARNING,
              'error': logging.ERROR,
              'critical': logging.CRITICAL,
              }
    DEBUG_NAMES = ['critical', 'error', 'warning', 'info', 'debug']

    def __init__(self):
        pass

    @property
    def debug(self):
        pass

    @debug.setter
    def debug(self, debug):
        pass

    def run_command(self, cmd):
        pass

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class fileDB(object):
    """A file based database.
    A file based database, read and write arguements in the specific file.
    """
    def __init__(self, db:str, mode:str=None, owner:str=None):  
        '''Init the db_file is a file to save the datas.'''
        pass


    def file_check_create(self, file_path:str, mode:str=None, owner:str=None):
        pass
    
    def get(self, name, default_value=None):
        """Get value by data's name. Default value is for the arguemants do not exist"""
        return default_value
    
    def set(self, name, value):
        """Set value by data's name. Or create one if the arguement does not exist"""
        pass




class Servo(_Basic_class):
    def __init__(self, pwm):
        pass

    # angle ranges -90 to 90 degrees
    def angle(self, angle):
        pass

    # pwm_value ranges MIN_PW 500 to MAX_PW 2500 degrees
    def set_pwm(self,pwm_value):
        pass


class I2C(_Basic_class):
    def __init__(self, *args, **kargs):     # *args表示位置参数（形式参数），可无，； **kargs表示默认值参数，可无。
        pass

    def _i2c_write_byte(self, addr, data):   # i2C 写系列函数
        pass
    
    def _i2c_write_byte_data(self, addr, reg, data):
        pass
    
    def _i2c_write_word_data(self, addr, reg, data):
        pass
    
    def _i2c_write_i2c_block_data(self, addr, reg, data):
        pass
    
    def _i2c_read_byte(self, addr):   # i2C 读系列函数
        pass

    def _i2c_read_i2c_block_data(self, addr, reg, num):
        pass

    def is_ready(self, addr):
        pass

    def scan(self):                             # 查看有哪些i2c设备
        pass

    def send(self, send, addr, timeout=0):                      # 发送数据，addr为从机地址，send为数据
        pass

    def recv(self, recv, addr=0x00, timeout=0):     # 接收数据
        pass

    def mem_write(self, data, addr, memaddr, timeout=5000, addr_size=8): #memaddr match to chn
        pass
    
    def mem_read(self, data, addr, memaddr, timeout=5000, addr_size=8):     # 读取数据
        pass
    
    def readfrom_mem_into(self, addr, memaddr, buf):
        pass
    
    def writeto_mem(self, addr, memaddr, data):
        pass

class PWM(I2C):
    def __init__(self, channel, debug="critical"):
        pass

    def i2c_write(self, reg, value):
        pass

    def freq(self, *freq):
        pass

    def prescaler(self, *prescaler):
        pass

    def period(self, *arr):
        pass

    def pulse_width(self, *pulse_width):
        pass

    def pulse_width_percent(self, *pulse_width_percent):
        pass



class Pin(_Basic_class):
    PULL_NONE = None
    def __init__(self, *value):
        pass
        
    def check_board_type(self):
        pass

    def init(self, mode, pull):
        pass

    def dict(self, *_dict):
        pass

    def __call__(self, value):
        return self.value(value)

    def value(self, *value):
        return value

    def on(self):
        return self.value(1)

    def off(self):
        return self.value(0)

    def high(self):
        return self.on()

    def low(self):
        return self.off()

    def mode(self, *value):
        pass

    def pull(self, *value):
        pass

    def irq(self, handler=None, trigger=None, bouncetime=200):
        pass

    def name(self):
        pass

    def names(self):
        return [self.name]

    class cpu(object):
        def __init__(self):
            pass

class ADC(I2C):
    ADDR=0x14                   # 扩展板的地址为0x14

    def __init__(self, chn):    # 参数，通道数，树莓派扩展板上有8个adc通道分别为"A0, A1, A2, A3, A4, A5, A6, A7"
        pass
        
    def read(self):                     # adc通道读取数---写一次数据，读取两次数据 （读取的数据范围是0~4095）
        pass

    def read_voltage(self):                             # 将读取的数据转化为电压值（0~3.3V）
        pass

class Grayscale_Module(object):
    def __init__(self, pin0, pin1, pin2, reference=1000):
        pass

    def get_line_status(self,fl_list):
        return "no status"

    def get_grayscale_data(self):
        return []

class Ultrasonic():
    def __init__(self, trig, echo, timeout=0.02):
        pass

    def _read(self):
        return -1

    def read(self, times=10):
        return -1
    

def run_command(cmd):
    import subprocess
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result = p.stdout.read().decode('utf-8')
    status = p.poll()
    return status, result


def reset_mcu():
    mcu_reset = Pin("MCURST")
    mcu_reset.off()
    time.sleep(0.001)
    mcu_reset.on() 
    time.sleep(0.01)  