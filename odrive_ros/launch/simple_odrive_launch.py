"""Simple launch that runs the minimal odrive node by executing the script.

This launch uses ExecuteProcess to run the node script with the current Python
interpreter. It is intended for development and quick tests; for production,
install the package and use a Node action instead.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import sys


def generate_launch_description():
    pkg = 'odrive_ros'
    node_script = os.path.join(get_package_share_directory(pkg), 'odrive_ros_node.py')
    cmd = [sys.executable, node_script]
    return LaunchDescription([
        ExecuteProcess(cmd=cmd, output='screen')
    ])
