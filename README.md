# MAV Cable-Suspended Object-Lifting
 
This repository contains the ROS nodes for a Bebop 2 to lift a bucket-shaped load with a hook attached at the end of a cable.

[Here](https://youtu.be/rvh3BWcd1Pg) you can see a lift by detecting only the position and orientation of the load.
[Here](https://youtu.be/5vbkehoSd-Y) you can see a lift by detecting the position of the load and the position of the hook.
[Here](https://youtu.be/LRMbYODOS-k) you can see a lift by detecting the position and orientation of the load, as well as the position of the hook.


## Nodes Description

**control_frontal** Main control, shows video feed, and executes position and lifting control policies.
**bebop_autonomy** ROS driver for Bebop 2, publishes the video feed and receives commands control.
**obj_loc** Bucket detector.
**ssd_grasper** Hook detector.
**teclado** Keyboard commands interface.