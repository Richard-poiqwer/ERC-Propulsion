"""
Enhanced ODrive ROS2 node (packaged)

Features added compared to standalone script:
- Packaged as a ROS2 Python package entry point
- Supports velocity mode and position mode with a ROS service to switch modes
- Subscribes to `/wheel_vel_cmds` (Float64MultiArray) for velocity commands (rad/s)
- Subscribes to `/wheel_pos_cmds` (Float64MultiArray) for position commands (rad) in position mode
- Publishes `/joint_states` for RViz visualization
- Supports an emergency stop via `/estop` (std_msgs/Bool)
- Service `/set_position_mode` (std_srvs/SetBool) where True enables position mode, False sets velocity mode

Assumptions:
- ODrive devices have been calibrated/homed when using position control.
"""

from __future__ import annotations

import math
import time
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

try:
    import odrive
    # import enums used by ODrive Python API v0.5.6
    from odrive.enums import *  # noqa: F401,F403
except Exception:
    odrive = None
    # enums will be undefined if import failed; code checks existence before use


class ODriveMotor:
    def __init__(self, serial: Optional[str] = None, axis: int = 0):
        self.serial = serial
        self.axis_index = int(axis)
        self.drive = None
        self.axis = None
        self.available = False

        # state
        self.command_rad_s = 0.0
        self.command_pos_rad = 0.0
        self.pos_rad = 0.0
        self.vel_rad_s = 0.0
        self._last_read_time = None

        self._connect()
        # motor-level flags (configured by node when switching modes)
        self.circular_setpoints = False
        self.using_trap_traj = False
        self.pos_filter_bandwidth = None

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
            revs_per_s = self.command_rad_s / (2.0 * math.pi)
            self.axis.controller.input_vel = revs_per_s
        except Exception:
            self.available = False

    def set_position_rad(self, rad: float):
        self.command_pos_rad = float(rad)
        if not self.available:
            return
        try:
            revs = self.command_pos_rad / (2.0 * math.pi)
            # If circular setpoints are enabled on the controller, input_pos should be in [0,1)
            try:
                circ = bool(getattr(self.axis.controller.config, 'circular_setpoints', False))
            except Exception:
                circ = False

            if circ:
                # use the fractional part only (one turn range)
                frac = revs % 1.0
                self.axis.controller.input_pos = frac
            else:
                # input_pos expects revolutions
                self.axis.controller.input_pos = revs
        except Exception:
            self.available = False

    def stop(self):
        try:
            if self.available:
                self.axis.controller.input_vel = 0.0
                self.axis.requested_state = AxisState.IDLE
        except Exception:
            pass

    def read_state(self):
        now = time.time()
        dt = None
        if self._last_read_time is not None:
            dt = now - self._last_read_time
        self._last_read_time = now

        if not self.available:
            self._connect()
            return

        try:
            # prefer circular position if controller is using circular setpoints
            try:
                circ = bool(getattr(self.axis.controller.config, 'circular_setpoints', False))
            except Exception:
                circ = False

            if circ:
                pos = getattr(self.axis.encoder, 'pos_circular', None)
            else:
                pos = getattr(self.axis.encoder, 'pos_estimate', None)

            vel = getattr(self.axis.encoder, 'vel_estimate', None)

            if pos is not None:
                # encoder pos is in turns; convert to radians
                try:
                    self.pos_rad = float(pos) * 2.0 * math.pi
                except Exception:
                    self.pos_rad = float(pos)

            if vel is not None:
                try:
                    self.vel_rad_s = float(vel) * 2.0 * math.pi
                except Exception:
                    self.vel_rad_s = float(vel)

            if pos is None and vel is not None and dt is not None:
                self.pos_rad += self.vel_rad_s * dt

        except Exception:
            self.available = False
            return


class ODriveRosNode(Node):
    def __init__(self):
        super().__init__('odrive_ros_node')

        # params
        self.declare_parameter('motors', [])
        self.declare_parameter('joint_names', [])
        self.declare_parameter('axis', 0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('command_topic', '/wheel_vel_cmds')
        self.declare_parameter('pos_command_topic', '/wheel_pos_cmds')
        self.declare_parameter('joint_state_topic', '/joint_states')
        # control configuration parameters
        self.declare_parameter('circular_setpoints', False)
        self.declare_parameter('use_trap_traj', False)
        self.declare_parameter('pos_filter_bandwidth', 0.0)
        self.declare_parameter('trap_traj_vel_limit', 1.0)
        self.declare_parameter('trap_traj_accel', 1.0)
        self.declare_parameter('trap_traj_decel', 1.0)
        self.declare_parameter('vel_ramp_rate', 0.0)

        motors_param = self.get_parameter('motors').get_parameter_value().string_array_value
        joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        axis_param = int(self.get_parameter('axis').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        cmd_topic = str(self.get_parameter('command_topic').value)
        pos_cmd_topic = str(self.get_parameter('pos_command_topic').value)
        js_topic = str(self.get_parameter('joint_state_topic').value)

        if len(motors_param) == 0:
            self.get_logger().info('No motors configured via parameter `motors` - using 4 autodiscover placeholders')

        # read control config params (used when switching modes)
        self._param_circular = bool(self.get_parameter('circular_setpoints').value)
        self._param_trap = bool(self.get_parameter('use_trap_traj').value)
        self._param_pos_filter_bw = float(self.get_parameter('pos_filter_bandwidth').value)
        self._param_trap_vel = float(self.get_parameter('trap_traj_vel_limit').value)
        self._param_trap_accel = float(self.get_parameter('trap_traj_accel').value)
        self._param_trap_decel = float(self.get_parameter('trap_traj_decel').value)
        self._param_vel_ramp = float(self.get_parameter('vel_ramp_rate').value)

        self.motors: List[ODriveMotor] = []
        for m in motors_param:
            if ':' in m:
                serial, ax = m.split(':', 1)
                self.motors.append(ODriveMotor(serial=serial.strip(), axis=int(ax)))
            else:
                self.motors.append(ODriveMotor(serial=m.strip(), axis=axis_param))

        if len(self.motors) == 0:
            for _ in range(4):
                self.motors.append(ODriveMotor(serial=None, axis=axis_param))

        if len(joint_names) != len(self.motors):
            joint_names = [f"wheel_{i}_joint" for i in range(len(self.motors))]

        self.joint_names = joint_names

        # publishers / subscribers / services
        self.js_pub = self.create_publisher(JointState, js_topic, 10)
        self.cmd_sub = self.create_subscription(Float64MultiArray, cmd_topic, self.cmd_callback, 10)
        self.pos_cmd_sub = self.create_subscription(Float64MultiArray, pos_cmd_topic, self.pos_cmd_callback, 10)
        self.estop_sub = self.create_subscription(Bool, '/estop', self.estop_callback, 10)
        self.mode_srv = self.create_service(SetBool, '/set_position_mode', self.set_position_mode_srv)

        self._cmd_lock = threading.Lock()
        self._pending_cmds = [0.0] * len(self.motors)
        self._pending_pos_cmds = [0.0] * len(self.motors)

        self._position_mode = False

        period = 1.0 / max(1.0, self.publish_rate)
        self.timer = self.create_timer(period, self._update)

        self.get_logger().info(f'ODrive ROS node started: {len(self.motors)} motors, publishing {js_topic} at {self.publish_rate} Hz')

    def cmd_callback(self, msg: Float64MultiArray):
        data = list(msg.data)
        with self._cmd_lock:
            for i in range(min(len(data), len(self._pending_cmds))):
                self._pending_cmds[i] = float(data[i])

    def pos_cmd_callback(self, msg: Float64MultiArray):
        data = list(msg.data)
        with self._cmd_lock:
            for i in range(min(len(data), len(self._pending_pos_cmds))):
                self._pending_pos_cmds[i] = float(data[i])

    def estop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('E-STOP received: stopping all motors immediately')
            for m in self.motors:
                try:
                    m.stop()
                except Exception:
                    pass

    def set_position_mode_srv(self, request: SetBool.Request, response: SetBool.Response):
        """Set or clear position mode. request.data == True => enable position mode."""
        enable = bool(request.data)
        ok = True
        msg = ''
        for m in self.motors:
            if not m.available:
                # attempt to connect; proceed regardless
                m._connect()
            try:
                # Apply control mode (uses enums from odrive.enums)
                if 'CONTROL_MODE_POSITION_CONTROL' in globals() and 'CONTROL_MODE_VELOCITY_CONTROL' in globals():
                    m.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL if enable else CONTROL_MODE_VELOCITY_CONTROL

                # Configure input mode and auxiliary settings based on parameters
                if enable:
                    # Position mode
                    # circular setpoints
                    try:
                        m.axis.controller.config.circular_setpoints = bool(self._param_circular)
                    except Exception:
                        pass

                    # trajectory vs filter vs passthrough
                    if self._param_trap and 'INPUT_MODE_TRAP_TRAJ' in globals():
                        try:
                            m.axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
                            m.axis.trap_traj.config.vel_limit = float(self._param_trap_vel)
                            m.axis.trap_traj.config.accel_limit = float(self._param_trap_accel)
                            m.axis.trap_traj.config.decel_limit = float(self._param_trap_decel)
                        except Exception:
                            pass
                    elif self._param_pos_filter_bw > 0 and 'INPUT_MODE_POS_FILTER' in globals():
                        try:
                            m.axis.controller.config.input_mode = INPUT_MODE_POS_FILTER
                            m.axis.controller.config.input_filter_bandwidth = float(self._param_pos_filter_bw)
                        except Exception:
                            pass
                    else:
                        if 'INPUT_MODE_PASSTHROUGH' in globals():
                            try:
                                m.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                            except Exception:
                                pass
                else:
                    # Velocity mode
                    if 'INPUT_MODE_VEL_RAMP' in globals() and self._param_vel_ramp > 0:
                        try:
                            m.axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
                            m.axis.controller.config.vel_ramp_rate = float(self._param_vel_ramp)
                        except Exception:
                            pass
                    else:
                        if 'INPUT_MODE_PASSTHROUGH' in globals():
                            try:
                                m.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                            except Exception:
                                pass
            except Exception as e:
                ok = False
                msg += f'Failed to set mode on a motor: {e}; '

        self._position_mode = enable
        response.success = ok
        response.message = 'position_mode=' + str(enable) + ('; ' + msg if msg else '')
        return response

    def _update(self):
        with self._cmd_lock:
            cmds = list(self._pending_cmds)
            pos_cmds = list(self._pending_pos_cmds)

        # apply commands according to mode
        for i, m in enumerate(self.motors):
            try:
                if self._position_mode:
                    # expect radians
                    if i < len(pos_cmds):
                        m.set_position_rad(pos_cmds[i])
                else:
                    if i < len(cmds):
                        m.set_velocity_rad_s(cmds[i])
            except Exception:
                pass

        # read and publish states
        for m in self.motors:
            m.read_state()

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        js.position = [m.pos_rad for m in self.motors]
        js.velocity = [m.vel_rad_s for m in self.motors]
        self.js_pub.publish(js)

    def destroy_node(self):
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
