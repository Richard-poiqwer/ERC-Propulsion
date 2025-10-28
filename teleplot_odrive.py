#!/usr/bin/env python3
"""teleplot_odrive.py

Send ODrive telemetry (target vs actual velocity, bus current, motor Iq)
to a teleplot-compatible UDP server at 10 Hz.

Usage examples:
    python teleplot_odrive.py                # connect to hardware and send telemetry
    python teleplot_odrive.py --addr 127.0.0.1:47269

The script sends messages of the form: name:timestamp_ms:value|g
which matches the example in the user's prompt.
"""
from __future__ import print_function

import socket
import time
import argparse
# math/random were used only by the previous simulate mode and are not needed now


import odrive
from odrive.enums import AxisState



def send_telemetry(sock, addr, name, value):
    now = int(time.time() * 1000)
    msg = f"{name}:{now}:{value}|g"
    try:
        sock.sendto(msg.encode(), addr)
    except Exception:
        # non-fatal - often happens if server not listening
        pass


def find_drives(serial_numbers=None, timeout=3.0):
    """Try to find ODrive drives. Returns list (possibly empty).

    If odrive library is not available, returns empty list.
    """
    if odrive is None:
        return []

    drives = []
    if serial_numbers:
        # try each serial quickly
        for ser in serial_numbers:
            try:
                d = odrive.find_any(serial_number=ser)
                if d is not None:
                    drives.append(d)
            except Exception:
                pass
    else:
        # try to find any (non-blocking alternative not provided by API);
        # use find_any once and return if found
        try:
            d = odrive.find_any()
            if d is not None:
                drives.append(d)
        except Exception:
            pass

    return drives


def sample_and_send(drives, sock, addr, tick=0):
    """Sample telemetry values (real drives) and send via UDP."""

    # real hardware: sample first drive.axis0 for now (extendable)
    # we send target_v and actual_v for each drive index
    for i, d in enumerate(drives):
        # safe attribute access - some reads can fail if drive disconnected
        
        target = float(d.axis0.controller.input_vel)        
        actual = float(d.axis0.encoder.vel_estimate)        
        bus_current = float(d.ibus)        
        motor_current = float(d.axis0.motor.current_control.Iq_measured)        
        v_int_d = float(d.axis0.motor.current_control.v_current_control_integral_d)        
        v_int_q = float(d.axis0.motor.current_control.v_current_control_integral_q)        
        bus_voltage = float(d.vbus_voltage)
        electrical_power = float(d.axis0.controller.electrical_power)
        mechanical_power = float(d.axis0.controller.mechanical_power)

        send_telemetry(sock, addr, f"drive{i}_target_vel", round(target, 4))
        send_telemetry(sock, addr, f"drive{i}_actual_vel", round(actual, 4))
        send_telemetry(sock, addr, f"drive{i}_bus_current", round(bus_current, 4))
        send_telemetry(sock, addr, f"drive{i}_motor_current", round(motor_current, 4))
        send_telemetry(sock, addr, f"drive{i}_v_current_int_d", round(v_int_d, 6))
        send_telemetry(sock, addr, f"drive{i}_v_current_int_q", round(v_int_q, 6))
        send_telemetry(sock, addr, f"drive{i}_bus_voltage", round(bus_voltage, 4))
        send_telemetry(sock, addr, f"drive{i}_electrical_power", round(electrical_power, 4))
        send_telemetry(sock, addr, f"drive{i}_mechanical_power", round(mechanical_power, 4))


def main():
    parser = argparse.ArgumentParser(description="Send ODrive telemetry to teleplot UDP server at 10 Hz.")
    parser.add_argument("--addr", default="127.0.0.1:47269",
                        help="teleplot UDP address HOST:PORT (default 127.0.0.1:47269)")
    parser.add_argument("--serial", nargs="*",
                        help="optional serial numbers to find (space separated)")
    args = parser.parse_args()

    host, port = args.addr.split(":")
    addr = (host, int(port))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Require the odrive library and at least one connected drive.
    drives = []
    if odrive is None:
        raise SystemExit("ODrive python library not available. Install the `odrive` package and try again.")

    print("Looking for ODrive drives...")
    drives = find_drives(args.serial)
    if len(drives) == 0:
        raise SystemExit("No ODrive drives found. Connect hardware or provide serial numbers.")

    print(f"Found {len(drives)} drive(s). Sampling axis0 on each drive.")
    # try to set closed loop on axis0 (non-fatal)
    for d in drives:
        try:
            if AxisState is not None:
                d.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        except Exception:
            pass

    print(f"Sending telemetry to {addr[0]}:{addr[1]} at 10 Hz. Ctrl-C to quit.")
    tick = 0
    period = 0.1  # 10 Hz
    try:
        while True:
            start = time.time()
            sample_and_send(drives, sock, addr, tick=tick)
            tick += 1
            # sleep to maintain ~10Hz
            elapsed = time.time() - start
            to_sleep = period - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)
    except KeyboardInterrupt:
        print("\nInterrupted, stopping telemetry send.")
    finally:
        # if we talked to drives, try to zero inputs and set to IDLE
        if drives:
            for i, d in enumerate(drives):
                try:
                    d.axis0.controller.input_vel = 0
                    if AxisState is not None:
                        d.axis0.requested_state = AxisState.IDLE
                    print(f"Stopped drive[{i}]")
                except Exception:
                    pass


if __name__ == '__main__':
    main()
