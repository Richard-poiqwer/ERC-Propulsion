# ERC Propulsion

Repository for ODrive-based hub-motor control and ROS integration for a small rover.
This project contains utility scripts, configuration files, and a packaged ROS2 node
(`odrive_ros`) to control multiple ODrive axes and publish joint states for RViz.

## Contents 
- `config.json` — default hoverboard ODrive settings (for H1/H2 motor).  
- `calibrate.py` — helper script for calibration steps (use on a free-spinning motor only).
- `multi_velocity.py` — simple script to test multiple drives with velocity commands.
- `teleplot_odrive.py` — plots ODrive telemetry (works with Teleplot VSCode extension).
- `odrive_ros/` — ROS2 Python package with the packaged node, launch, URDF, and README.

## Prerequisites
- ODrive v3.6 (and derivatives) with [firmware version 0.5.6](https://docs.odriverobotics.com/releases/firmware), others not supported
- [Update using STM32CubeProgrammer](https://ffbeast.github.io/docs/en/software_firmware_flashing.html). Hold BOOT then press RESET, then connect over USB
- ODrive package: `pip install odrive` (currently using version 0.6.10.post0)

## Quick start
1. Connect the ODrive to your PC (USB) and confirm it appears in `odrivetool`. Note the serial number!  
2. For a new motor other than the 6" H1/H2, follow the hoverboard setup guide. Note that for MKS Odrive, instead of `odrv0` it will appear as `dev0` due to it not being a genuine device. Otherwise, exit and restore the config:  
```bash
odrivetool restore-config config.json
```
3. Run calibration (set serial numbers of motors to calibrate first!)
```bash
python calibration.py
```
4. Run the simple velocity tester =:
```bash
python velocity.py
```
5. Test multiple motors (must set serial numbers of connected motors) =:
```bash
python multi_velocity.py
```

## ROS usage and visualization
The repository includes a ROS2 package in `odrive_ros/` which exposes a node that
publishes `/joint_states`, accepts velocity/position commands, and supports
incremental moves. For ROS-specific usage, parameter details and launch examples,
see: `odrive_ros/README.md` 

## Links and documentation
- [ODrive firmware v0.5.6 docs (DON'T USE LATEST!)](https://docs.odriverobotics.com/v/0.5.6/)
- [Hoverboard setup (follow for new motors!)](https://docs.odriverobotics.com/v/0.5.6/hoverboard.html)
- [API Documentation](https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html)
- [Tuning Guide](https://docs.odriverobotics.com/v/0.5.6/control.html )

## Safety and tuning notes

- Use `odrivetool` to limit currents, voltages and speeds; sensible limits are
	already stored in `config.json`.
- Important limits to consider:
	- max DC input current (protect PSU/battery)
	- max DC output current (regen braking)
	- max motor current (thermal safety)
	- max speed and max DC voltage
- The motor thermistor should ideally be configured to trigger overheat
	shutdown: https://docs.odriverobotics.com/v/0.5.6/thermistors.html#thermistor-coefficients  
- Position-control requires calibrated motors
