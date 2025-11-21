import rclpy
from rclpy.node import Node
import rclpy.utilities
import rclpy.executors
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data

import odrive
from odrive.enums import AxisState, InputMode

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from odrive_ros.config.mappings import AXES
from odrive_ros.config.network import baseQoS, stillQoS
from odrive_ros.config.serial import drives

import time
from dataclasses import dataclass
from typing import List

@dataclass
class twist:
    linear: float
    rotation: float


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
    
    @property
    def speed(self):
        if self.drive is None:
            return

        return float(self.drive.axis0.encoder.vel_estimate) * self.polarity      


# REQUIRES BASE_PING NODE TO OPERATE MANUALLY 
class TelepresenceOperations(Node):
    def __init__(self, scale=3.5, ramp_rate=3.0):
        super().__init__("teleop")

        self.mappings = []
        for e in drives:
            serial = e['serial']
            side = e['side']
            polarity = e['polarity']
            self.mappings.append(DriveMapping(serial, side, polarity))

        self.find_drives(self.mappings)#

        for item in self.mappings:
            item.drive.axis0.controller.config.input_mode = InputMode.VEL_RAMP
            item.drive.axis0.controller.config.vel_ramp_rate = ramp_rate

        node_cb_group = MutuallyExclusiveCallbackGroup()
        connection_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Scale factor to convert stick (-1...1) to rev/s
        self.scale = scale

        # Topics
        self.controller_commands_sub_ = self.create_subscription(
            Joy,
            "/joy",
            self.teleopCB_,
            qos_profile=qos_profile_sensor_data,
            callback_group=node_cb_group,
        )
        self.base_ping_sub_ = self.create_subscription(
            Bool,
            "/ping",
            self.confirmConnectionCB_,
            qos_profile=baseQoS,
            callback_group=connection_cb_group,
        )
        # Publishers
        self.state_still_pub_ = self.create_publisher(
            Bool, "/gorgon/still", qos_profile=stillQoS
        )

        self.velocity_pub_ = self.create_publisher(
                Twist, "/wheel_vel", qos_profile=qos_profile_sensor_data
        )

         # State -
        self.state = twist(0, 0)
        self.target = twist(0, 0)

        self.stationary = False

        # Connection timer
        self.last_connection_ = time.monotonic()
        self.connection_timer_ = self.create_timer(0.5, self.shutdownCB_, node_cb_group)
        self.driver_timer_ = self.create_timer(0.02, self.driveCB_, node_cb_group)
        self.vel_timer_ = self.create_timer(0.1, self.velCB_, node_cb_group)

        # Servo Offset control
        # Temporary Variable
        OFFSET = 0
        self.offset_ = OFFSET
        # Temporary Seperation
        self.wheel_seperation_ = 0.4

        # wheel_seperation, scale and ramp_rate should all be ros params

    # -------------

    def confirmConnectionCB_(self, msg: Bool):
        self.last_connection_ = time.monotonic()

    def shutdownCB_(self):
        if time.monotonic() > self.last_connection_ + 1.2:
            self.get_logger().warn("Lost connection, setting movement to zero.")
            self.target.linear = 0
            self.target.rotation = 0
            self.drive()
    

    def teleopCB_(self, msg: Joy):
        # DRIVE -----------------
        # joystick is inverted from what you would expect
        self.target.linear = -msg.axes[AXES["TRIGGERRIGHT"]]
        self.target.linear += msg.axes[AXES["TRIGGERLEFT"]]
        # goes from 1 to -1, therefore difference between the two
        # should be halved.
        self.target.linear /= 2
        self.target.rotation = msg.axes[AXES["LEFTX"]]
        # ------------------------

        state = Bool()
        if (
            self.target.linear == 0
            and self.target.rotation != 0
            and not self.stationary
        ):
            self.stationary = True
            state.data = True
            self.state_still_pub_.publish(state)
        elif self.target.linear != 0 and self.stationary:
            self.stationary = False
            state.data = False
            self.state_still_pub_.publish(state)

    def driveCB_(self):
        self.drive()
    
    def drive(self):
        right_side = self.bound_range(self.target.linear + 0.5 * self.target.rotation) * self.scale
        left_side = self.bound_range(self.target.linear - 0.5 * self.target.rotation) * self.scale

        for m in self.mappings:
            m.apply_speed(left_side, right_side)

        # self.get_logger().info(
        #     "left_side: " + str(left_side) + " right_side: " + str(right_side)
        # )


    def velCB_(self):
        linear_vel, angular_vel = self.current_twist()
        msg = Twist()
        msg.linear.y = linear_vel
        msg.angular.z = angular_vel

        self.velocity_pub_.publish(msg)
       

    def current_twist(self):
        linear_vel = 0.0
        angular_vel = 0.0
        for drive in self.mappings:
            wheel_speed = drive.speed 
            linear_vel += wheel_speed
            if drive.side == "left":
                angular_vel -= wheel_speed
            elif drive.side == "right":
                angular_vel += wheel_speed

        linear_vel /= 4
        angular_vel /= 2 * self.wheel_seperation_

        return [linear_vel, angular_vel]


    @staticmethod
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

    @staticmethod
    def bound_range(value):
        if value > 1:
            value = 1
        elif value < -1:
            value = -1
        return value



def main(args=None):
    rclpy.init(args=args)

    tele = TelepresenceOperations()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tele)
    try:
        executor.spin()
    except KeyboardInterrupt:
        tele.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        tele.destroy_node()
        rclpy.utilities.try_shutdown()
