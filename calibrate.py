#!/usr/bin/env python3
"""
Calibrate the odrive and motor with hoverboard configuration
"""

from __future__ import print_function

import odrive
from odrive.enums import *
# DeviceLostException is raised when the USB connection is lost (e.g. device reboots)
from odrive.libodrive import DeviceLostException
import odrive.utils
import odrive.legacy_config
import time
import json
import math

# will block if it can't find one of these
serial_numbers = [
    # "3471346D3034", # i
    # "348B34663034", # ii
    "346A34583034", # iii
    # "346E34613034", # iv
]

# conf=json.load("config.json")

# Find a connected ODrive (this will block until you connect one)

for ser in serial_numbers:
    d = odrive.find_any(serial_number=ser)
    if d is None:
        print(f"Warning: could not find drive with serial '{ser}'")

    print("Restoring config")
    # disable pre-configured on reboot incase previous config failed

    try:
        odrive.legacy_config.restore_config_ui(d,"config.json")
    except DeviceLostException:
        pass

    time.sleep(4)
    d = odrive.find_any(serial_number=ser)
    if d is None:
        print(f"Failed to reconnect to '{ser}'")

    print("Clearing calibration")
    d.axis0.encoder.config.pre_calibrated = False 
    d.axis0.motor.config.pre_calibrated = False
    d.axis0.config.startup_closed_loop_control = False 

    try:
        d.save_configuration()
    except DeviceLostException:
        print("device disconnected during reboot (expected)")
    except Exception as e:
        print("unexpected error during reboot:", e)

    time.sleep(3)
    d = odrive.find_any(serial_number=ser)
    if d is None:
        print(f"Warning: could not find drive with serial '{ser}'")

    # Calibrate motor and wait for it to finish
    print("starting motor calibration...")
    d.axis0.requested_state = AxisState.MOTOR_CALIBRATION 
    while d.axis0.current_state != AxisState.IDLE:
        time.sleep(0.1)

    if d.axis0.error==0x00:
        print("success")

        print("starting encoder calibration...")
        # skip, assume calibrated! seems to break for some reason...
        d.axis0.encoder.config.hall_polarity_calibrated=True

        # d.axis0.requested_state = AxisState.ENCODER_HALL_POLARITY_CALIBRATION 
        # while d.axis0.current_state != AxisState.IDLE:
        #     time.sleep(0.1)

        if d.axis0.encoder.error==0x00:
            print("success")

            print("starting encoder offset calibration...")
            d.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION 
            while d.axis0.current_state != AxisState.IDLE:
                time.sleep(0.1)

            if d.axis0.encoder.error==0x00:
                print("success")

                d.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

                #save the calibration
                d.axis0.encoder.config.pre_calibrated = True 
                d.axis0.motor.config.pre_calibrated = True
                d.axis0.config.startup_closed_loop_control = True # enable closed loop control on reboot

                print("calibration saved")
                d.axis0.requested_state = AxisState.IDLE # set idle to allow config save and reboot
                
                try:
                    d.save_configuration()
                    print("configuration saved")
                except DeviceLostException:
                    # The device often disconnects immediately when saving or
                    # rebooting. Treat this as an expected behaviour and move on.
                    print("device disconnected during save (expected) — configuration likely saved")
                    continue
                except Exception as e:
                    print("unexpected error saving configuration:", e)
                    continue

                try:
                    d.reboot()
                except DeviceLostException:
                    print("device disconnected during reboot (expected)")
                    continue
                except Exception as e:
                    print("unexpected error during reboot:", e)
                    continue

                continue # skip error message
                #! Add extra calibration for position control and estimation, cogging etc
                #! Add setup for acceleration etc

                #! add a way to save these values so we don't have to redo on each motor and can easily restore
    # print detailed error enums (instead of raw numbers) for axis, drive and encoder
    def _format_enum(enum_cls, value):
        """Return a human-readable name for an enum.IntFlag/Enum value.

        Examples:
          AxisError(1) -> 'AxisError.INVALID_STATE'
          EncoderError(3) -> 'EncoderError.SOME_FLAG|EncoderError.ANOTHER_FLAG'
        Falls back to the numeric value if conversion fails.
        """
        if value is None:
            return 'None'
        try:
            enum_val = enum_cls(value)
        except Exception:
            return str(value)

        # If a single-name exists, use it
        name = getattr(enum_val, 'name', None)
        if name:
            return f"{enum_cls.__name__}.{name}"

        # IntFlag with combined bits: list all matching members (except zero)
        parts = []
        for member in enum_cls:
            try:
                if member.value != 0 and (member.value & int(enum_val)) == member.value:
                    parts.append(f"{enum_cls.__name__}.{member.name}")
            except Exception:
                # skip any member that can't be compared
                continue
        if parts:
            return '|'.join(parts)

        # fallback to numeric
        return str(int(enum_val))

    axis_err = _format_enum(AxisError, d.axis0.error)
    enc_err = _format_enum(EncoderError, d.axis0.encoder.error)
    motor_err = _format_enum(MotorError, d.axis0.motor.error)
    # the device-level error may not always be present; fall back to attribute if missing
    try:
        drive_err = _format_enum(ODriveError, d.error)
    except Exception:
        drive_err = _format_enum(ODriveError, getattr(d, 'error', None))

    print("calibration failed! trying next motor")
    print(f"  axis error   : {axis_err}")
    print(f"  motor error  : {motor_err}")
    print(f"  encoder error: {enc_err}")
    print(f"  drive error  : {drive_err}")
    

