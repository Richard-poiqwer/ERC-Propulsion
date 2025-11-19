from __future__ import print_function

import odrive
from odrive.enums import *
import time

# add the serial numbers here (shows when you connect using odrivetool CLI)
# will hang if it can't find one
serial_numbers = [
    "3471346D3034", # i
    "348B34663034", # ii
    "346A34583034", # iii
    "346E34613034", # iv
]

drives = []
for ser in serial_numbers:
    d = odrive.find_any(serial_number=ser)
    if d is None:
        print(f"Warning: could not find drive with serial '{ser}'")
    else:
        drives.append(d)

if len(drives) == 0:
    raise SystemExit("No ODrive drives found. Check connections and serial numbers.")

# initialize closed loop control for axis0 on all found drives
for drive in drives:
    try:
        drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    except Exception as e:
        print(f"Failed to set CLOSED_LOOP_CONTROL on drive: {e}")

print("Drives available:")
for i, d in enumerate(drives):
    try:
        sn = d.serial_number
    except Exception:
        sn = "<unknown>"
    print(f"  [{i}] serial: {sn}")

print("\nUsage: enter '<motor_index> <velocity>' (e.g. '0 3.5') to set axis0 velocity in rev/s")
print("Enter 'q' or Ctrl-C to quit.")

try:
    while True:
        line = input("motor_index velocity> ")
        if line is None:
            continue
        line = line.strip()
        if line.lower() in ("q", "quit", "exit"):
            break

        parts = line.split()
        if len(parts) != 2:
            print("Expected two values: <motor_index> <velocity>")
            continue

        try:
            idx = int(parts[0])
        except ValueError:
            print("Motor index must be an integer")
            continue

        try:
            vel = float(parts[1])
        except ValueError:
            print("Velocity must be a number (rev/s)")
            continue

        if idx < 0 or idx >= len(drives):
            print(f"Invalid motor_index {idx}. Available range: 0..{len(drives)-1}")
            continue

        drive = drives[idx]
        try:
            drive.axis0.controller.input_vel = vel
            print(f"Set drive[{idx}].axis0.controller.input_vel = {vel}")
        except Exception as e:
            print(f"Failed to set velocity on drive {idx}: {e}")

        # small pause to allow the odrive to process commands
        time.sleep(0.01)
except KeyboardInterrupt:
    print("\nInterrupted by user")
finally:
    # on exit, try to stop all motors and set them to IDLE
    for i, d in enumerate(drives):
        try:
            d.axis0.controller.input_vel = 0
            d.axis0.requested_state = AxisState.IDLE
            print(f"Stopped drive[{i}]")
        except Exception:
            pass