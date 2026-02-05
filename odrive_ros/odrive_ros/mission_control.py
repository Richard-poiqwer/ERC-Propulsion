import rclpy
from rclpy.node import Node
from rclpy.parameter import get_parameter_value
import rclpy.utilities
import rclpy.executors
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data

# import odrive
# from odrive.enums import AxisState, InputMode

#from std_msgs.msg import Bool, Header
#from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistWithCovariance, Vector3
#from nav_msgs.msg import Odometry

from odrive_ros.config.mappings import AXES
# from odrive_ros.config.network import baseQoS
#from odrive_ros.config.serial import drives

import time
from dataclasses import dataclass
from typing import List
import numpy as np

# @dataclass
# class twist:
#     linear: float
#     rotation: float

########################### MissionControl ###########################

class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")

        # Subscriptions 
        self.controller_commands_sub_ = self.create_subscription(
            Joy,
            "/joy",
            self.teleopCB_,
            qos_profile=qos_profile_sensor_data,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        # Publishers
        self.pubtwist = self.create_publisher(
                Motion, "/cmd_vel", qos_profile=qos_profile_sensor_data
        )
    
############################# Functions #############################
  
    def teleopCB_(self, msg: Joy): 
        # DRIVE -----------------
        # joystick is inverted from what you would expect
        self.target.linear = -msg.axes[AXES["TRIGGERRIGHT"]] 
        self.target.linear += msg.axes[AXES["TRIGGERLEFT"]] 
        # goes from 1 to -1, therefore difference between the two
        # should be halved.
        self.target.linear /= 2 
        self.target.rotation = msg.axes[AXES["LEFTX"]] 

        pubtwist_msg = Motion()
        pubtwist_msg.motion = Twist(
                    linear=Vector3(
                        x=self.trarget.linear,
                        y=float(0),
                        z=float(0),
                    ),
                    angular=Vector3(
                        x=float(0),
                        y=float(0),
                        z=self.target.rotation
                    )
        )
        self.pubtwist.publish(pubtwist_msg)
        self.get_logger().info('Publishing: "%d"' % pubtwist_msg.motion)


