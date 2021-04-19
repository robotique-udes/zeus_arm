# Zeus arm
This package contains all the necessary code to operate the physical arm module or the gazebo simulation. To launch the simulation, open a terminal and enter the following command

```
roslaunch zeus_arm simulation.launch
```

To control the physical arm (not functional yet), the command is 

```
roslaunch zeus_arm teleop.launch
```

The main difference between the simulation and the actual robot is the low-level control. On the actual arm, the Arduino Mega 2560 passes the joint velocity commands through a PI 
control loop and sends the resulting commands to the motor drivers for each joint. The Arduino also reads the encoders positions and sends them back to the arm's onboard PC (Jetson Nano)
In the simulation, this part is handled by the ros_control package, which simulates fake controllers for the robot's joints.



