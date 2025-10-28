"""Launch file to start odrive_ros_node and robot_state_publisher with a URDF.

Adjust parameters below or pass on the command line.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('odrive_ros')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf')

    odrive_node = Node(
        package='odrive_ros',
        executable='odrive_ros_node',
        name='odrive_ros_node',
        output='screen',
        parameters=[{
            'motors': [],
            'joint_names': [],
            'publish_rate': 50.0,
            'command_topic': '/wheel_vel_cmds',
            'pos_command_topic': '/wheel_pos_cmds',
            'joint_state_topic': '/joint_states'
        }]
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    return LaunchDescription([odrive_node, rsp])
