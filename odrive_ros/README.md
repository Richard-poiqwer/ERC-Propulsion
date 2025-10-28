# odrive_ros

ROS 2 Python package that exposes multiple ODrive axes to ROS: velocity and
position control, incremental moves, and joint-state publishing for RViz.

Features
- Velocity control (rad/s) via topic `/wheel_vel_cmds` (Float64MultiArray)
- Position control (rad) via `/wheel_pos_cmds` (Float64MultiArray) and
  service `/set_position_mode`
- Incremental moves via `/wheel_move_incremental` (Float64MultiArray)
- Publishes `/joint_states` for visualization (URDF + robot_state_publisher)
- Emergency stop via `/estop` (std_msgs/Bool)

Dependencies
- ROS 2 (Jazzy)
- `rclpy`, `robot_state_publisher`, `sensor_msgs`, `std_msgs`, `std_srvs`

Quick development run (no ROS packaging)
```bash
# from the workspace root
python odrive_ros/odrive_ros_node.py
```

Recommended ROS 2 usage
1) Put the `odrive_ros` folder into a ROS 2 workspace (src/).
2) Install Python deps (example):
```bash
pip install odrive
```
3) Build and source the workspace:
```bash
# from workspace root
colcon build --symlink-install
source install/setup.bash
```
4) Launch with the provided launch file:
```bash
ros2 launch odrive_ros odrive_launch.py
```

Launch-time parameter overrides
You can override parameters at launch time with `--ros-args -p` or supply a params
YAML file. Example overriding the motors and enabling circular setpoints:
```bash
ros2 launch odrive_ros odrive_launch.py \
  --ros-args -p motors:="['odrv0:0','odrv0:1','odrv1:0','odrv1:1']" \
  -p joint_names:="['wheel_fl_joint','wheel_fr_joint','wheel_rl_joint','wheel_rr_joint']" \
  -p circular_setpoints:=True -p use_trap_traj:=True -p trap_traj_vel_limit:=2.0
```

Topics and services
- /wheel_vel_cmds (Float64MultiArray): velocity commands in rad/s per-wheel.
- /wheel_pos_cmds (Float64MultiArray): absolute position commands in radians per-wheel.
- /wheel_move_incremental (Float64MultiArray): delta position in radians applied as an incremental move.
  - The node will call the ODrive native `move_incremental` if available, otherwise
    it computes and sets a new `input_pos` using encoder feedback (respects `circular_setpoints`).
- /joint_states (sensor_msgs/JointState): publishes current joint position (rad) and velocity (rad/s).
- /estop (std_msgs/Bool): when True, stops motors immediately.
- /set_position_mode (std_srvs/SetBool): set True to enable position mode, False for velocity mode.

Examples

Publish a 4-wheel velocity command (rad/s):
```bash
ros2 topic pub /wheel_vel_cmds std_msgs/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0]"
```

Switch to position mode:
```bash
ros2 service call /set_position_mode std_srvs/srv/SetBool "data: true"
```

Send an incremental move (+0.1 rad to wheel 0):
```bash
ros2 topic pub /wheel_move_incremental std_msgs/Float64MultiArray "data: [0.1, 0.0, 0.0, 0.0]"
```

RViz visualization
1) Ensure the `urdf/rover.urdf` is installed or robot_description parameter is provided (the launch does this).
2) Run `ros2 launch odrive_ros odrive_launch.py`.
3) Open RViz, add a TF and RobotModel display to see wheel rotations from `/joint_states`.
