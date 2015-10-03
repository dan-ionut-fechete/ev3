from ev3.lego import MediumMotor
from ev3.ev3dev import Motor
import unittest
import time
from util import get_input


class TesMediumMotor(unittest.TestCase):
    def test_medium_motor(self):
        get_input('Attach a MediumMotor then continue')
        d = MediumMotor()
        print(d.type)
        #d.run_forever(25, regulation_mode=False)
	d.run_time_limited(time_sp=5000, speed_sp=25, regulation_mode=False,
                           stop_mode=Motor.STOP_MODE.COAST, ramp_up_sp=100, ramp_down_sp=100)
        print(d.run)
        time.sleep(5)
        d.stop()
if __name__ == '__main__':
    unittest.main()
