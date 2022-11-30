# Files descritpion
The following subsections will give a brief summary of each file and its role.

#### cam.launch
Launch file for the usb_cam node. Image dimensions and video device can be modified inside this launch file.

#### bringup.launch
Common launch file for either the simulation or the real arm. This launch file starts nodes that are required for both situations.

#### teleop.launch
Launch file for the real robot. In summary, this file :
- Runs the common launch file.
- Launches the teleoperation node and the arm node.
