import rclpy
from rclpy.node import Node
from rclpy.parameter import get_parameter_value
import rclpy.utilities
import rclpy.executors
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

from odrive_ros.config.mappings import AXES

import time
from dataclasses import dataclass
import numpy as np

@dataclass
class twist:
    linear: float
    rotation: float

########################### MissionControl ###########################

class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")

        self.declare_parameter("speed", 1.0) # float (turn/s)
        self.declare_parameter("wheel_radius", 0.08) # float

        # Scale factor to convert stick (-1...1) to m/s and rads/s
        self.speed_max = 2 * np.pi * self.get_parameter("wheel_radius").value * self.get_parameter("speed").value
        self.angular_speed_max = 2 * np.pi * self.get_parameter("speed").value

        # Subscriptions 
        self.controller_commands_sub_ = self.create_subscription(
            Joy,
            "/joy",
            self.teleopCB_,
            qos_profile=qos_profile_sensor_data, # Mutually exclusive callback vs Multithreaded executors?
        )
        # Publishers
        self.pubtwist = self.create_publisher(
                Twist, "/cmd_vel", qos_profile=qos_profile_sensor_data
        )
    
############################# Functions #############################
  
    def teleopCB_(self, msg: Joy): 
        # DRIVE -----------------
        self.target = twist(0, 0)
        # joystick is inverted from what you would expect
        self.target.linear = -msg.axes[AXES["TRIGGERRIGHT"]] 
        self.target.linear += msg.axes[AXES["TRIGGERLEFT"]] 
        # goes from 1 to -1, therefore difference between the two
        # should be halved.
        self.target.linear /= 2 
        self.target.rotation = msg.axes[AXES["LEFTX"]] 

        self.target.linear = self.target.linear * self.speed_max
        self.target.rotation = self.target.rotation * self.angular_speed_max

        pubtwist_msg = Twist(
                    linear=Vector3(
                        x=self.target.linear,
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
