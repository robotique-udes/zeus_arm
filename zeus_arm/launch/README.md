# Files descritpion
The following subsections will give a brief summary of each file and its role.

## cam.launch
Launch file for the usb_cam node. Image dimensions and video device can be modified inside this launch file.

## bringup.launch
Common launch file for either the simulation or the real arm. This launch file starts nodes that are required for both situations, such as the joystick node and the user interface.

## jetson.launch
Launch file to setup the multimaster structure.

## simulation.launch
Launch file for the gazebo simulation. In summary, this file : 
- Runs the common launch file;
- Launches an empty gazebo world and pauses the physics;
- Spawns the robot using the URDF file;
- Spawns the fake joint controllers;
- Launches the node to monitor joint states (position, velocity and acceleration);
- Launches the teleoperation node and the arm node;
- Launches a script to set the gravity to 0 in Gazebo and the unpauses the physics engine. The gravity is turned off in the simulation since the real robot's linear joints are strong enough to hold it in place when the robot is idle. This way, there is no need to compute a gravity compensation in the control loop and the simulation behaves the same as the real robot. 

## teleop.launch
Launch file for the real robot. In summary, this file :
- Runs the common launch file;
- Launches the serial node from the rosserial package to allow communication between the onboard PC (Jetson Nano) and the Arduino Mega 2560 controlling the motors via USB connection.
- Launches the teleoperation node and the arm node.
