"""Very small ODrive ROS2 test node.

This minimal node subscribes to a Float64MultiArray on the configured
command topic (default: /wheel_vel_cmds). Each element is interpreted
as a wheel angular velocity in radians/second and applied directly to
the corresponding ODrive axis via controller.input_vel.

This node intentionally does not publish any state or offer services.
It's intended for quick manual tests only.
"""

from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

try:
    import odrive
    from odrive.enums import *  # noqa: F401,F403
except Exception:
    odrive = None


class ODriveMotor:
    """Tiny wrapper to connect and set velocity on an ODrive axis."""

    def __init__(self, serial: str | None = None, axis: int = 0):
        self.serial = serial
        self.axis_index = int(axis)
        self.drive = None
        self.axis = None
        self.available = False
        self._connect()

    def _connect(self):
        if odrive is None:
            return
        try:
            if self.serial:
                self.drive = odrive.find_any(serial_number=self.serial)
            else:
                self.drive = odrive.find_any()
            if self.drive is None:
                self.available = False
                return
            self.axis = self.drive.axis0 if self.axis_index == 0 else self.drive.axis1
            self.available = True
            # try to enable closed loop control if possible
            try:
                self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
            except Exception:
                pass
        except Exception:
            self.available = False

    def set_velocity_rad_s(self, rad_s: float) -> None:
        if not self.available:
            self._connect()
        if not self.available:
            return
        try:
            revs_per_s = float(rad_s) / (2.0 * math.pi)
            self.axis.controller.input_vel = revs_per_s
        except Exception:
            self.available = False

    def stop(self) -> None:
        try:
            if self.available:
                self.axis.controller.input_vel = 0.0
                self.axis.requested_state = AxisState.IDLE
        except Exception:
            pass


class ODriveRosNode(Node):
    """Minimal ROS2 node: subscribe to velocities and apply them.

    Parameters (ROS parameters):
    - motors: string[] optional list of motor serials or serial:axis entries
    - axis: default axis index if motors entries do not include axis
    - command_topic: topic to subscribe to (Float64MultiArray)
    """

    def __init__(self):
        super().__init__('odrive_ros_node_simple')

        self.declare_parameter('motors', [])
        self.declare_parameter('axis', 0)
        self.declare_parameter('command_topic', '/wheel_vel_cmds')

        motors_param = self.get_parameter('motors').get_parameter_value().string_array_value
        axis_param = int(self.get_parameter('axis').value)
        cmd_topic = str(self.get_parameter('command_topic').value)

        self.motors: list[ODriveMotor] = []
        for m in motors_param:
            if ':' in m:
                serial, ax = m.split(':', 1)
                self.motors.append(ODriveMotor(serial=serial.strip(), axis=int(ax)))
            else:
                self.motors.append(ODriveMotor(serial=m.strip() if m else None, axis=axis_param))

        if len(self.motors) == 0:
            # default to two motors if none configured
            self.motors = [ODriveMotor(None, axis_param) for _ in range(2)]

        self.create_subscription(Float64MultiArray, cmd_topic, self.cmd_callback, 10)
        self.get_logger().info(f'Simple ODrive node started, subscribed to {cmd_topic}')

    def cmd_callback(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        for i, v in enumerate(data):
            if i >= len(self.motors):
                break
            try:
                self.motors[i].set_velocity_rad_s(float(v))
            except Exception as e:
                self.get_logger().warn(f'failed to set velocity on motor {i}: {e}')

    def destroy_node(self) -> None:
        for m in self.motors:
            try:
                m.stop()
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ODriveRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
