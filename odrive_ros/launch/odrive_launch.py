"""Launch file to start `odrive_ros_node` and `robot_state_publisher` with a URDF.

Adjust parameters below or pass overrides at launch time.

Example (override parameters at launch time using ROS 2 parameter CLI):

ros2 launch odrive_ros odrive_launch.py \
    --ros-args -p motors:="['odrv0:0','odrv0:1','odrv1:0','odrv1:1']" \
    -p joint_names:="['wheel_fl_joint','wheel_fr_joint','wheel_rl_joint','wheel_rr_joint']" \
    -p circular_setpoints:=True -p use_trap_traj:=True -p trap_traj_vel_limit:=2.0

You can also supply parameters from a YAML file with `--ros-args --params-file <file>`.
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
            # Example defaults (override at launch):
            'motors': ['odrv0:0', 'odrv0:1', 'odrv1:0', 'odrv1:1'],
            'joint_names': ['wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint'],
            'publish_rate': 50.0,
            'command_topic': '/wheel_vel_cmds',
            'pos_command_topic': '/wheel_pos_cmds',
            'joint_state_topic': '/joint_states',
            # control tuning examples
            'circular_setpoints': True,
            'use_trap_traj': True,
            'trap_traj_vel_limit': 2.0,
            'trap_traj_accel': 5.0,
            'trap_traj_decel': 5.0,
            'pos_filter_bandwidth': 0.0,
            'vel_ramp_rate': 0.0
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
