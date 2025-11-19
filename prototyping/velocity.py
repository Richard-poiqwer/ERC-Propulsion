from __future__ import print_function

import odrive
from odrive.enums import *
import time

print("finding an odrive...")
my_drive = odrive.find_any()
my_drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL


while True:
    value=input("Enter a velocity in rev/s: ")
    try:
        value=float(value)
        my_drive.axis0.controller.input_vel = value
    except:
        my_drive.axis0.controller.input_vel = 0
        time.sleep(5)
        my_drive.axis0.requested_state = AxisState.IDLE