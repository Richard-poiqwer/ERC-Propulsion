# ERC Propulsion 

tools and ros node  

requires latest odrivetool  
try out the odrivetool CLI with a driver connected over USB-c  

all available settings at https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html   

Designed for MKS Odrive Mini  
Using official Odrive firmware v0.5.6  

initial setup should follow https://docs.odriverobotics.com/v/0.5.6/hoverboard.html  

Use calibrate.py on a free spinning motor only  
More calibration for cogging etc, position control may be required  

`multi_velocity.py` is an easy way to test the drives. need to comment out serial numbers of drives that aren't connected  
`teleplot_odrive.py` will plot various params to teleplot vscode extension, extend as required. sadly official odrive UI incompatible.  

# ROS node

currently a vibe coded pile of trash  
aim is to have it take JointTrajectory inputs and output JointStates to allow rviz use  
eventually should support position control!