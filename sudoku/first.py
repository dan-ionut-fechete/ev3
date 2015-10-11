from ev3.ev3dev import Motor
from ev3.lego import MediumMotor
from ev3.lego import LargeMotor
from ev3.lego import ColorSensor

from sudoku.matrix import SudokuMatrix

import time
import os
import logging

class SmartColorSensor(ColorSensor):
    """ColorSensor class that avoid opening a file everytime
        a value needs to be read.
        Doesn't work :(
    """    

    def __init__(self):
        ColorSensor.__init__(self)
        self.value0_path = os.path.join(self.sys_path, 'value0')
        print(self.value0_path)
        self.value0_file = open(self.value0_path)

    def read_value0(self):
        value = self.value0_file.read().strip()
        return value
    
class RotatingMotor(object):
    """ Helper methods for motors rotating to exact degrees.
    """

    def __init__(self, motor):
        self.rotating_motor = motor
        print("Motor position: ", self.rotating_motor.position)
        print("Motor state: ", self.rotating_motor.state)
        print("Position mode: ", self.rotating_motor.position_mode)

    def rotate_to(self, degrees, speed, func=None):
        """
            Rotates degrees at speed and executes func if provided continuously 
            while rotating.
        """
        print('Rotating to', degrees)
        start = time.time()

        self.rotating_motor.run_position_limited(
                position_sp=degrees,
                speed_sp=speed,
                stop_mode=Motor.STOP_MODE.HOLD,
                ramp_up_sp=0,
                ramp_down_sp=0, 
                regulation_mode='on'
        )

        k_timelimit = 5

        while (self.rotating_motor.state != 'idle' 
                and time.time() - start < k_timelimit):
            if func is not None:
                func()

        if (time.time() - start > k_timelimit):
            logging.exception("Timeout while trying to rotate to ", degrees)

    @property
    def state(self):
        return self.rotating_motor.state

    @property
    def position(self):
        return self.rotating_motor.position

class Scanner(object):
    """ Class controlling the scanner of a SudokuRobot
    """

    # Number of degreese to scan to the left and to the right of the center 
    k_degrees_to_scan = 60
    # How fast to scan
    k_scan_speed = 120
    # Expected number of data points per scan. Due to the accuracy of the
    # motors or sensor (or lack thereof). We might get a higher or lower
    # number of samples we therefore need to fit those samples to the matrix
    # and average the values. This number most likely will come from experimenting
    # with the robot.
    k_expected_samples_per_scan=145

    def __init__(self):
        self.rotating_motor = RotatingMotor(MediumMotor())
        self.color_sensor = ColorSensor()

    def setup(self):
        # Get ready to scan.
        self.rotating_motor.rotate_to(self.k_degrees_to_scan, self.k_scan_speed)

    def reset(self):        
        """Set the scanning motor to point straight ahead.
        """
        self.rotating_motor.rotate_to(0, self.k_scan_speed)

    def scan(self):
        logging.info("Scanning")
        start_pos = 0
        new_position = start_pos - self.k_degrees_to_scan
        if self.rotating_motor.position < 0:
            new_position = start_pos + self.k_degrees_to_scan
        
        colors = []
        positions = []

        # Set the mode to reflect
        self.color_sensor.mode = 'COL-REFLECT'

        self.rotating_motor.rotate_to(
            new_position, 
            self.k_scan_speed,
            lambda col=colors, scan=self: col.append(scan.color_sensor.reflect)
        )

        print(str(len(colors)) + " scans")
        return colors

class Steering(object):
    """ Class used for steering the sudoku robot.
    """
    k_speed = 80

    def __init__(self):
        self.motor = RotatingMotor(LargeMotor(port='A'))

    def reset(self):
        self.motor.rotate_to(0, self.k_speed / 2)

    def left(self):
        self.motor.rotate_to(-45, self.k_speed)

    def right(self):
        self.motor.rotate_to(45, self.k_speed)

class Driver(object):
    k_one_unit = -4
    # Current position, we assume we always start at 0.
    x = 0
    y = 0


    """Class controling the position of a SudokuRobot.
        Controls one large motor.
    """
    def __init__(self):
        self.motor = LargeMotor(port='D')

    def set_start_position(self):
        # Sets current position as the start.
        self.x = self.y = 0

    def reset(self):
        # Moves to the starting position.
        self.move(-self.y)

    def move(self, units): 
        speed = 30

        if abs(self.k_one_unit * units) > 40:
            speed = 120
        self.motor.run_position_limited(
                position_sp=self.k_one_unit * units, 
                speed_sp=speed,
                stop_mode=Motor.STOP_MODE.HOLD, 
                ramp_up_sp=0, 
                ramp_down_sp=0, 
                regulation_mode='on',
                position_mode='relative'
        )

        time.sleep(1)
        self.y += units        

    # TODO Add seeting stuff, right 45, move -275? or more, left -45, move, straighten, move back.

class SudokuRobot(object):
    def __init__(self):
        print("init")
        self.scanner = Scanner()
        # Set robot in position to start scanning.
        self.scanner.setup()

        self.driver = Driver()

    def move_forward(self):
        self.driver.move(1)

    def get_coords(self):
        return (self.driver.x, self.driver.y)

    def one_scan(self):
        return self.scanner.scan()

    def reset(self):        
        """Set the scanning motor to point straight ahead.
        """
        self.scanner.reset()
        self.driver.reset()


def test_scanning():
    s = SudokuRobot()    
    num_scans = 0
    for i in range(0, 5):    
        num_scans += len(s.one_scan())
        # Sleep a bit between scans.
        time.sleep(0.2)
    print(num_scans / 5)
    s.reset()

def test_scan_and_matrix():
    m = SudokuMatrix()
    s = SudokuRobot()    
    meas = s.one_scan()
    s.reset()
    m.add_scan(0, 0, 120, meas)

def test_move_scan_and_matrix():
    m = SudokuMatrix()
    s = SudokuRobot()

    for i in range(0, 160):
        s.move_forward()
        print("Moved to ", s.get_coords())
        time.sleep(0.1)
        for j in range(0, 2):
            print("Scan ", j)
            meas = s.one_scan()
            if j % 2:
                meas = meas[::-1]
            coords = s.get_coords()
            m.add_scan(coords[0], coords[1], 120, meas)
            time.sleep(0.1)            
    s.reset()
    m.dump_to_file('slim_right.txt')

def test_one_scan_matrix():
    m = SudokuMatrix()
    meas = []
    for i in range(0, 145):
        meas.append(i)

    m.add_scan(0, 0, 120, meas)

def test_moving_forward():
    s = SudokuRobot()
    for i in range(0, 10):
        s.move_forward()
        time.sleep(0.2)
    s.reset()

def test_moving():
    s = SudokuRobot()
    for i in range(0, 10):
        s.move_forward()
        time.sleep(0.2)
        for j in range(0, 4):
            s.one_scan()
            time.sleep(0.2)
    s.reset()

def reset_all():
    s = SudokuRobot()
    s.reset()

def test_sudoku_matrix():
    m = SudokuMatrix()
    m.load_from_file('matrixv1.txt')
    m.add_scan(0, 0, 120, [1, 2, 3, 4])
    m.dump_to_file('matrixv1.txt')

def health_check():
    s = Scanner()
    
if __name__ == '__main__':
    test_move_scan_and_matrix()


