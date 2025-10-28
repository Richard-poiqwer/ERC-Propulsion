# odrive_ros

ROS2 Python package wrapping an ODrive multi-motor node.

Features
- Velocity control (rad/s)
- Position control (rad) via mode switch
- Publishes /joint_states for RViz
- Emergency stop support

Quick test (standalone):

Run the node directly (for quick dev):
```bash
python odrive_ros/odrive_ros_node.py
```

Recommended (ROS2):
- Install package dependencies (odrive library, ROS2 installed on machine)
- Build as ament package or use `ros2 run` after installing the package properly
