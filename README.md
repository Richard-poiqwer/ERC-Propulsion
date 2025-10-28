# ERC Propulsion

Repository for ODrive-based hub-motor control and ROS integration for a small rover.
This project contains utility scripts, configuration files, and a packaged ROS2 node
(`odrive_ros`) to control multiple ODrive axes and publish joint states for RViz.

Contents (selected)
- `config.json` — default ODrive settings used with `odrivetool restore-config`.
- `calibrate.py` — helper script for calibration steps (use on a free-spinning motor only).
- `multi_velocity.py` — simple script to test multiple drives with velocity commands.
- `teleplot_odrive.py` — plots ODrive telemetry (works with Teleplot VSCode extension).
- `odrive_ros/` — ROS2 Python package with the packaged node, launch, URDF, and README.

Prerequisites
- ODrive tools and library (this repo targets drivers with ODrive firmware v0.5.6)
    - Python package: `pip install odrive`
- ROS 2 (if you want to use the `odrive_ros` node and RViz visualization).

Quick start (non-ROS, development)
1. Connect an ODrive to your PC (USB) and confirm it appears in `odrivetool`.
2. Restore or tweak configuration if needed:
```bash
odrivetool restore-config config.json
```
3. Run the simple multi-velocity tester (edit serials as needed in the script):
```bash
python multi_velocity.py
```

ROS usage and visualization
The repository includes a ROS2 package in `odrive_ros/` which exposes a node that
publishes `/joint_states`, accepts velocity/position commands, and supports
incremental moves. For ROS-specific usage, parameter details and launch examples,
see: `odrive_ros/README.md` (it contains usage, launch overrides, topic examples,
and RViz instructions).

Links and documentation
- ODrive API and firmware v0.5.6 docs: https://docs.odriverobotics.com/v/0.5.6/

Safety and tuning notes
- Use `odrivetool` to limit currents, voltages and speeds; sensible limits are
	already stored in `config.json` (review before powering motors).
- Important limits to consider:
	- max DC input current (protect PSU/battery)
	- max DC output current (regen braking)
	- max motor current (thermal safety)
	- max speed and max DC voltage
- The motor thermistor (if present) should be configured to trigger overheat
	shutdown. See the thermistor docs for beta values and configuration.
- Position-control requires calibrated motors — the node assumes
	calibrated axes when position mode is enabled.
