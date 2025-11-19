"""Remote control TCP server for ODrive motors.

Listens for newline-terminated JSON messages. Example message:
  {"left": 1.2, "right": -1.2, "triggers_held": true}

Server maps incoming `left`/`right` speeds to configured drives. Each drive
entry in the config must provide a serial number, a side ("left" or "right")
and a polarity (1 or -1) which multiplies the incoming speed.

Create a local config file (see `rc_server_config.example.json`) or pass --config.
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from typing import Dict, List

try:
    import odrive
    from odrive.enums import AxisState
except Exception:
    odrive = None


def load_config(path: str) -> Dict:
    with open(path, 'r') as f:
        return json.load(f)


class DriveMapping:
    def __init__(self, serial: str, side: str, polarity: int):
        self.serial = serial
        self.side = side.lower()
        self.polarity = int(polarity)
        self.drive = None

    def apply_speed(self, left: float, right: float):
        if self.drive is None:
            return
        if self.side == 'left':
            v = left * self.polarity
        else:
            v = right * self.polarity
        try:
            self.drive.axis0.controller.input_vel = v
        except Exception as e:
            print(f"Failed to set velocity on {self.serial}: {e}")


def find_drives(mappings: List[DriveMapping]):
    if odrive is None:
        print("odrive library not available; running in dry-run mode.")
        return

    for m in mappings:
        try:
            d = odrive.find_any(serial_number=m.serial)
        except Exception as e:
            print(f"Error finding drive {m.serial}: {e}")
            d = None
        if d is None:
            print(f"Warning: could not find drive with serial '{m.serial}'")
        else:
            m.drive = d
            # try to ensure axis0 is in closed loop
            try:
                d.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
            except Exception:
                pass


def handle_message(payload: Dict, mappings: List[DriveMapping]):
    left = float(payload.get('left', 0.0))
    right = float(payload.get('right', 0.0))
    triggers = bool(payload.get('triggers_held', False))

    if not triggers:
        left = 0.0
        right = 0.0

    for m in mappings:
        m.apply_speed(left, right)


def client_thread(conn: socket.socket, addr, mappings: List[DriveMapping]):
    print(f"Client connected: {addr}")
    conn.settimeout(1.0)
    buffer = b''
    try:
        while True:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue
            if not data:
                break
            buffer += data
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                if not line:
                    continue
                try:
                    payload = json.loads(line.decode('utf-8'))
                except Exception as e:
                    print(f"Invalid JSON from {addr}: {e}")
                    continue
                handle_message(payload, mappings)
    finally:
        try:
            conn.close()
        except Exception:
            pass
        print(f"Client disconnected: {addr}")


def run_server(host: str, port: int, mappings: List[DriveMapping]):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(5)
    print(f"RC server listening on {host}:{port}")

    try:
        while True:
            conn, addr = sock.accept()
            t = threading.Thread(target=client_thread, args=(conn, addr, mappings), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print("Shutting down server")
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='rc_server_config.json', help='Path to server config JSON')
    args = parser.parse_args()

    try:
        cfg = load_config(args.config)
    except FileNotFoundError:
        print(f"Config {args.config} not found. Copy rc_server_config.example.json to rc_server_config.json and edit serials.")
        return

    host = cfg.get('host', '0.0.0.0')
    port = int(cfg.get('port', 9000))
    drv_entries = cfg.get('drives', [])
    mappings = []
    for e in drv_entries:
        serial = e.get('serial')
        side = e.get('side', 'left')
        polarity = e.get('polarity', 1)
        mappings.append(DriveMapping(serial, side, polarity))

    find_drives(mappings)

    run_server(host, port, mappings)


if __name__ == '__main__':
    main()
