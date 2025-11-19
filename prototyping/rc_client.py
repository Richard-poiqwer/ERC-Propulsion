"""Remote control client that reads a gamepad and sends JSON speed commands.

Default behavior maps two vertical stick axes to left/right wheel velocities.
Both triggers must be held for motion; otherwise the client sends zero speeds.

This uses pygame for joystick input. Configure axes/buttons via CLI flags.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time

try:
    import pygame
except Exception:
    pygame = None


def clamp(v, lo=-9999, hi=9999):
    return max(lo, min(hi, v))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=9000)
    parser.add_argument('--left-axis', type=int, default=1, help='Axis index for left stick vertical')
    parser.add_argument('--right-axis', type=int, default=3, help='Axis index for right stick vertical')
    parser.add_argument('--trigger-mode', choices=['buttons','axes'], default='axes')
    parser.add_argument('--lt', type=int, default=4, help='Left trigger button or axis index')
    parser.add_argument('--rt', type=int, default=5, help='Right trigger button or axis index')
    parser.add_argument('--axis-threshold', type=float, default=0.5, help='Axis threshold for triggers when using axes mode')
    parser.add_argument('--scale', type=float, default=3.5, help='Scale factor to convert stick (-1..1) to rev/s')
    parser.add_argument('--rate', type=float, default=20.0, help='Send rate (Hz)')
    args = parser.parse_args()

    if pygame is None:
        print('pygame is required for joystick support. Install with: pip install pygame')
        return

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print('No joystick found. Connect a controller and try again.')
        return

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f'Using joystick: {joy.get_name()} with {joy.get_numaxes()} axes and {joy.get_numbuttons()} buttons')

    try:
        sock = socket.create_connection((args.host, args.port), timeout=5.0)
    except Exception as e:
        print(f'Failed to connect to {args.host}:{args.port}: {e}')
        return

    period = 1.0 / float(args.rate)
    try:
        while True:
            start = time.time()
            pygame.event.pump()

            # read stick axes (vertical axis typically: up = -1, down = +1)
            try:
                ly = joy.get_axis(args.left_axis)
            except Exception:
                ly = 0.0
            try:
                ry = joy.get_axis(args.right_axis)
            except Exception:
                ry = 0.0

            # --- NEW: print live joystick values (axes and buttons) in-place ---
            try:
                axes = [round(joy.get_axis(i), 3) for i in range(joy.get_numaxes())]
            except Exception:
                axes = []
            try:
                buttons = [int(joy.get_button(i)) for i in range(joy.get_numbuttons())]
            except Exception:
                buttons = []
            # write a single updating line (carriage return) and flush
            # try:
            #     sys.stdout.write(f'\rAxes: {axes}  Buttons: {buttons}    ')
            #     sys.stdout.flush()
            # except Exception:
            #     pass
            # --- end new code ---

            # invert so that up => positive speed
            left_speed = -float(ly) * args.scale
            right_speed = -float(ry) * args.scale

            # triggers: either buttons or axes
            if args.trigger_mode == 'buttons':
                try:
                    lt_pressed = bool(joy.get_button(args.lt))
                    rt_pressed = bool(joy.get_button(args.rt))
                except Exception:
                    lt_pressed = False
                    rt_pressed = False
            else:
                try:
                    lt_val = joy.get_axis(args.lt)
                    rt_val = joy.get_axis(args.rt)
                except Exception:
                    lt_val = 0.0
                    rt_val = 0.0
                lt_pressed = abs(lt_val) > args.axis_threshold
                rt_pressed = abs(rt_val) > args.axis_threshold

            triggers_held = lt_pressed and rt_pressed

            if not triggers_held:
                left_speed = 0.0
                right_speed = 0.0

            left_speed = clamp(left_speed)
            right_speed = clamp(right_speed)

            payload = {'left': left_speed, 'right': right_speed, 'triggers_held': triggers_held}
            msg = json.dumps(payload).encode('utf-8') + b'\n'
            try:
                sock.sendall(msg)
            except Exception as e:
                print(f'Connection lost: {e}')
                break

            # sleep to maintain rate
            elapsed = time.time() - start
            to_sleep = period - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        try:
            sock.close()
        except Exception:
            pass
        pygame.joystick.quit()
        pygame.quit()


if __name__ == '__main__':
    main()
