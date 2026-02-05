import rclpy
from rclpy.node import Node
from rclpy.parameter import get_parameter_value
import rclpy.utilities
import rclpy.executors
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data

import odrive
from odrive.enums import AxisState, InputMode

#from std_msgs.msg import Bool, Header
#from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistWithCovariance, Vector3
#from nav_msgs.msg import Odometry

from odrive_ros.config.mappings import AXES
from odrive_ros.config.network import baseQoS
#from odrive_ros.config.serial import drives

import time
from dataclasses import dataclass
from typing import List
import numpy as np

@dataclass
class twist:
    linear: float
    rotation: float

########################### MissionControl ###########################

class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")

        node_cb_group = MutuallyExclusiveCallbackGroup()
        connection_cb_group = MutuallyExclusiveCallbackGroup()
        

        # Subscriptions 
        self.controller_commands_sub_ = self.create_subscription(
            Joy,
            "/joy",
            self.teleopCB_,
            qos_profile=qos_profile_sensor_data,
            callback_group=node_cb_group,
        )
        # self.base_ping_sub_ = self.create_subscription(
        #     Bool,
        #     "/ping",
        #     self.confirmConnectionCB_,
        #     qos_profile=baseQoS,
        #     callback_group=connection_cb_group,
        # )
        # Publishers
        self.cmd_vel = self.create_publisher(
                ------, "/cmd_vel", qos_profile=qos_profile_sensor_data
        )

         # State -
        self.state = twist(0, 0)
        self.target = twist(0, 0)

        self.stationary = False

        # Connection timer


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

