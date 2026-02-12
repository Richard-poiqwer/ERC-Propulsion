import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

from odrive_ros.config.mappings import AXES

from dataclasses import dataclass

@dataclass
class twist:
    linear: float
    rotation: float

########################### MissionControl ###########################

class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")
        self.target = twist(0, 0)

        # Scale factor to convert stick (-1...1) to m/s and rads/s
        # Directly arbitrarily defined
        self.speed_max = 0.6 # m/s
        self.angular_speed_max = 1.5 # rads/s

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
