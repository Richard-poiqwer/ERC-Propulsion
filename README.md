# ERC Propulsion 

tools and ros node for hoverboard motor control

requires latest odrivetool  
try out the odrivetool CLI with a driver connected over USB-c  

Default configs i've worked out are in config.json, you can use `odrivetool restore-config config.json`  

All available settings at https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html   

Designed for MKS Odrive Mini using any hoverboard motors
Using official Odrive firmware v0.5.6  

Initial setup of new motors should follow https://docs.odriverobotics.com/v/0.5.6/hoverboard.html  
The existing motors and drivers are already configured and calibrated.  


Use calibrate.py on a free spinning motor only  
More calibration for cogging etc, position control may be required  

Tuning: https://docs.odriverobotics.com/v/0.5.6/control.html

`multi_velocity.py` is an easy way to test the drives. need to comment out serial numbers of drives that aren't connected  
`teleplot_odrive.py` will plot various params to teleplot vscode extension, extend as required. sadly official odrive UI incompatible.  

# Safety
Odrivetool can be used to set the limits, these have already been set to reasonable values in config.json.
- max DC input current (to not overload the power supply / battery)
- max DC output current (regen braking, must be zero when on a power supply instead of a battery)
- max motor current (to avoid overheating)
- max speed
- max DC voltage (above which dump into resistor to avoid overvolting the battery)

The thermistor in the motor should also be configured to trigger an overheat shutdown, you'll have to guess the Beta value  
https://docs.odriverobotics.com/v/0.5.6/thermistors.html#thermistor-coefficients 

# ROS node

currently a vibe coded pile of trash  
aim is to have it take JointTrajectory inputs and output JointStates to allow rviz use  
eventually should support position control!