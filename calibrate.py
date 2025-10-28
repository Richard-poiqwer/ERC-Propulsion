#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
my_drive = odrive.find_any()

# Calibrate motor and wait for it to finish
print("starting motor calibration...")
my_drive.axis0.requested_state = AxisState.MOTOR_CALIBRATION #AxisState.FULL_CALIBRATION_SEQUENCE
while my_drive.axis0.current_state != AxisState.IDLE:
    time.sleep(0.1)

if my_drive.axis0.motor.error==0x00:
    print("success")

    print("starting encoder calibration...")
    my_drive.axis0.requested_state = AxisState.ENCODER_HALL_POLARITY_CALIBRATION #AxisState.FULL_CALIBRATION_SEQUENCE
    while my_drive.axis0.current_state != AxisState.IDLE:
        time.sleep(0.1)

    if my_drive.axis0.encoder.error==0x00:
        print("success")

        print("starting encoder offset calibration...")
        my_drive.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION #AxisState.FULL_CALIBRATION_SEQUENCE
        while my_drive.axis0.current_state != AxisState.IDLE:
            time.sleep(0.1)

        if my_drive.axis0.encoder.error==0x00:
            print("success")

            my_drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

            #save the calibration
            my_drive.axis0.encoder.config.pre_calibrated = True 
            my_drive.axis0.motor.config.pre_calibrated = True
            my_drive.axis0.config.startup_closed_loop_control = True # enable closed loop control on reboot

            print("calibration saved")
            my_drive.axis0.requested_state = AxisState.IDLE # set idle to allow config save and reboot
            
            my_drive.save_configuration()
            print("configuration saved")
            my_drive.reboot()


            #! Add extra calibration for position control and estimation, cogging etc
            #! Add setup for acceleration etc

            #! add a way to save these values so we don't have to redo on each motor and can easily restore