"""
odrive_ros_node.py

ROS2-compatible Python node to control multiple ODrive motors and publish joint_states
for RViz visualization.

Features:
- Connect to multiple ODrive devices (by serial or autodiscover)
- Support selecting axis (0 or 1) per motor
- Subscribe to `/wheel_vel_cmds` (std_msgs/Float64MultiArray) with velocities in rad/s
- Publish `/joint_states` (sensor_msgs/JointState) with position (rad) and velocity (rad/s)
- Graceful shutdown: stop motors and set IDLE

Notes:
- This node uses the `odrive` Python library (the same style as `multi_velocity.py`).
- Commands are converted from rad/s -> rev/s before sending to ODrive `controller.input_vel`.
- Encoder fields from ODrive are read if available; otherwise velocity-derived integration is used
  to estimate position for visualization.

Adjust parameters below or via ROS2 parameters when running.
"""

from __future__ import annotations

import math
import time
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

try:
    import odrive
    from odrive.enums import AxisState
except Exception as e:
    odrive = None
    AxisState = None


class ODriveMotor:
    def __init__(self, serial: Optional[str] = None, axis: int = 0):
        self.serial = serial
        self.axis_index = int(axis)
        self.drive = None
        self.axis = None
        self.available = False

        # state
        self.command_rad_s = 0.0
        self.pos_rad = 0.0
        self.vel_rad_s = 0.0
        self._last_read_time = None

        # attempt connection lazily
        self._connect()

    def _connect(self):
        if odrive is None:
            return
        try:
            if self.serial:
                self.drive = odrive.find_any(serial_number=self.serial)
            else:
                # find any attached drive
                self.drive = odrive.find_any()

            if self.drive is None:
                self.available = False
                return

            self.axis = self.drive.axis0 if self.axis_index == 0 else self.drive.axis1
            self.available = True
            # try to request closed loop if possible, but don't crash if it fails
            try:
                self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
            except Exception:
                pass
            self._last_read_time = time.time()
        except Exception:
            self.available = False

    def set_velocity_rad_s(self, rad_s: float):
        self.command_rad_s = float(rad_s)
        if not self.available:
            return
        try:
            # ODrive expects revolutions per second for input_vel (multi_velocity.py uses rev/s)
            revs_per_s = self.command_rad_s / (2.0 * math.pi)
            self.axis.controller.input_vel = revs_per_s
        except Exception:
            # mark unavailable and attempt reconnect next read
            self.available = False

    def stop(self):
        try:
            if self.available:
                self.axis.controller.input_vel = 0.0
                self.axis.requested_state = AxisState.IDLE
        except Exception:
            pass

    def read_state(self):
        """Read encoder/velocity from the ODrive and update pos_rad and vel_rad_s.

        If encoder fields are unavailable this method integrates the last commanded
        velocity to produce a position estimate for visualization.
        """
        now = time.time()
        dt = None
        if self._last_read_time is not None:
            dt = now - self._last_read_time
        self._last_read_time = now

        if not self.available:
            # try to connect again
            self._connect()
            return

        try:
            # ODrive encoder pos/vel fields are accessed if present
            pos_est = getattr(self.axis.encoder, 'pos_estimate', None)
            vel_est = getattr(self.axis.encoder, 'vel_estimate', None)

            if pos_est is not None:
                # pos_est often reports turns; convert to radians
                try:
                    self.pos_rad = float(pos_est) * 2.0 * math.pi
                except Exception:
                    # fallback if already in radians
                    self.pos_rad = float(pos_est)

            if vel_est is not None:
                try:
                    self.vel_rad_s = float(vel_est) * 2.0 * math.pi
                except Exception:
                    self.vel_rad_s = float(vel_est)

            # if pos not provided but vel is, integrate
            if pos_est is None and vel_est is not None and dt is not None:
                self.pos_rad += self.vel_rad_s * dt

        except Exception:
            # if read fails, mark unavailable and skip
            self.available = False
            return


class ODriveRosNode(Node):
    def __init__(self):
        super().__init__('odrive_ros_node')

        # Parameters
        self.declare_parameter('motors', [])
        self.declare_parameter('joint_names', [])
        self.declare_parameter('axis', 0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('command_topic', '/wheel_vel_cmds')
        self.declare_parameter('joint_state_topic', '/joint_states')

        motors_param = self.get_parameter('motors').get_parameter_value().string_array_value
        joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        axis_param = int(self.get_parameter('axis').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        cmd_topic = str(self.get_parameter('command_topic').value)
        js_topic = str(self.get_parameter('joint_state_topic').value)

        # If motors param empty, autodiscover - this will use any attached ODrive and axis 0
        if len(motors_param) == 0:
            self.get_logger().info('No motors configured via parameter `motors` - will autodiscover on startup (may take some seconds).')

        # Build motor objects
        self.motors: List[ODriveMotor] = []
        for m in motors_param:
            # allow motor entries like 'SERIAL:axis' or just 'SERIAL'
            if ':' in m:
                serial, ax = m.split(':', 1)
                self.motors.append(ODriveMotor(serial=serial.strip(), axis=int(ax)))
            else:
                self.motors.append(ODriveMotor(serial=m.strip(), axis=axis_param))

        # If no motors were configured, create placeholder motors that will attempt autodiscover
        if len(self.motors) == 0:
            # create 4 autodiscover placeholders (common for 4-wheel rover)
            for _ in range(4):
                self.motors.append(ODriveMotor(serial=None, axis=axis_param))

        # Joint names
        if len(joint_names) != len(self.motors):
            # default joint names
            joint_names = [f"wheel_{i}_joint" for i in range(len(self.motors))]

        self.joint_names = joint_names

        # Publisher and subscriber
        self.js_pub = self.create_publisher(JointState, js_topic, 10)
        self.cmd_sub = self.create_subscription(Float64MultiArray, cmd_topic, self.cmd_callback, 10)

        # Internal state for commands
        self._cmd_lock = threading.Lock()
        self._pending_cmds = [0.0] * len(self.motors)

        # Timer for read/publish and write cycle
        period = 1.0 / max(1.0, self.publish_rate)
        self.timer = self.create_timer(period, self._update)

        self.get_logger().info(f'ODrive ROS node started: {len(self.motors)} motors, publishing {js_topic} at {self.publish_rate} Hz')

    def cmd_callback(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) != len(self.motors):
            self.get_logger().warn(f'Received command length {len(data)} but expected {len(self.motors)}')
            # If shorter, ignore; if longer, slice
        # copy into pending commands
        with self._cmd_lock:
            for i in range(min(len(data), len(self._pending_cmds))):
                # expect radians per second from ROS world
                self._pending_cmds[i] = float(data[i])

    def _update(self):
        # 1) apply pending commands
        with self._cmd_lock:
            cmds = list(self._pending_cmds)

        for i, m in enumerate(self.motors):
            try:
                m.set_velocity_rad_s(cmds[i])
            except Exception:
                pass

        # 2) read states
        for m in self.motors:
            m.read_state()

        # 3) publish joint_states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        js.position = [m.pos_rad for m in self.motors]
        js.velocity = [m.vel_rad_s for m in self.motors]
        # effort left empty
        self.js_pub.publish(js)

    def destroy_node(self):
        # On shutdown, stop motors
        for m in self.motors:
            try:
                m.stop()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ODriveRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
