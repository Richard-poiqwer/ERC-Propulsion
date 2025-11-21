import rclpy
from rclpy.node import Node
import rclpy.utilities
import rclpy.executors
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data

import odrive
from odrive.enums import AxisState, InputMode

from std_msgs.msg import Bool, Float32MultiArray, Header
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry

from odrive_ros.config.mappings import AXES
from odrive_ros.config.network import baseQoS
from odrive_ros.config.serial import drives

import time
from dataclasses import dataclass
from typing import List
import numpy as np

@dataclass
class twist:
    linear: float
    rotation: float


class DriveMapping:
    def __init__(self, serial: str, side: str, polarity: int, wheel_radius: float):
        self.serial = serial
        self.side = side.lower()
        self.polarity = int(polarity)
        self.drive = None
        self.wheel_radius = wheel_radius

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
        # convert from turn/s to rads/s, then multiply by wheel_radius to return in m/s
        return float(2 * np.pi * self.drive.axis0.encoder.vel_estimate * self.polarity * self.wheel_radius) 


class TelepresenceOperations(Node):
    def __init__(self):
        super().__init__("teleop")

        self.declare_parameter("speed", 1.0) # float
        self.declare_parameter("ramp_rate", 1.0) # float
        self.declare_parameter("wheel_seperation", 0.4) # float
        self.declare_parameter("wheel_radius", 0.2) # float

        """
        PYRIGHT COMPLAINS: It seems function description is written incorrectly in the source. 
        self.declare_parameters(
                namespace="", 
                parameters=[("speed", 1.0), # float
                            ("ramp_rate", 1.0),  ("wheel_seperation", 0.4)]
            )
        """

        # Scale factor to convert stick (-1...1) to rev/s
        self.scale = self.get_parameter("speed").value
        
        # Set Wheel seperation for Odometry
        self.wheel_seperation_ = self.get_parameter("wheel_seperation").value


        self.mappings = []
        for e in drives:
            serial = e['serial']
            side = e['side']
            polarity = e['polarity']
            self.mappings.append(
                    DriveMapping(
                        serial,
                        side, 
                        polarity, 
                        self.get_parameter("wheel_radius").value # pyright: ignore
                        )
                    ) 

        self.find_drives(self.mappings)
    
        # Set Acceleration
        for d in self.mappings:
            d.drive.axis0.controller.config.input_mode = InputMode.VEL_RAMP
            d.drive.axis0.controller.config.vel_ramp_rate = self.get_parameter("ramp_rate").value

        node_cb_group = MutuallyExclusiveCallbackGroup()
        connection_cb_group = MutuallyExclusiveCallbackGroup()
        

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
        self.encoder_odom_pub_ = self.create_publisher(
                Odometry, "/gorgon/encoder/odom", qos_profile=qos_profile_sensor_data
        )

         # State -
        self.state = twist(0, 0)
        self.target = twist(0, 0)

        self.stationary = False

        # Connection timer
        self.last_connection_ = time.monotonic()
        self.connection_timer_ = self.create_timer(0.5, self.shutdownCB_, node_cb_group)
        self.driver_timer_ = self.create_timer(0.02, self.driveCB_, node_cb_group)
        self.odom_timer_ = self.create_timer(0.05, self.odomCB_, node_cb_group)

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

    def driveCB_(self):
        self.drive()
    
    def drive(self):
        right_side = self.bound_range(self.target.linear + 0.5 * self.target.rotation) * self.scale # pyright: ignore
        left_side = self.bound_range(self.target.linear - 0.5 * self.target.rotation) * self.scale # pyright: ignore

        for m in self.mappings:
            m.apply_speed(left_side, right_side)

        # self.get_logger().info(
        #     "left_side: " + str(left_side) + " right_side: " + str(right_side)
        # )


    def odomCB_(self):
        linear_vel, angular_vel = self.current_twist()

        # Place Holder Before Measuring Covariances
        cov_matrix = np.diag([
            0.1, # variance of x
            0.0, # variance of y
            0.0, # variance of z
            0.0, # variance of roll
            0.0, # variance of pitch
            0.1  # variance of yaw
        ])

        covariance = cov_matrix.flatten().tolist()

        odom_msg = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="encoder_odom",
            ),
            child_frame_id="base_link",
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=float(linear_vel),
                        y=float(0),
                        z=float(0),
                    ),
                    angular=Vector3(
                        x=float(0),
                        y=float(0),
                        z=float(angular_vel)
                    )
                ),
                covariance=Float32MultiArray(
                    data=covariance
                )
            ),
        )

        self.encoder_odom_pub_.publish(odom_msg)

       

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
        # Multiplied by 2, as double counting wheels, divided by 2, as radius of 
        # rotation is half of the diameter of rotation (Conver to radians, sucessfully)
        angular_vel /= self.wheel_seperation_ # pyright: ignore

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
