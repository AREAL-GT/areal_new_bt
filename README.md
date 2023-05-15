# New AREAL Behavior Tree ROS2 package

Documentation Page: https://areal-gt.github.io/documentation/_build/ROS%202%20Packages/areal_ROS2_pkg.html

This package contains an example behavior tree for the AREAL UAV.  The behavior tree is written in C++ and uses the BehaviorTree.CPP library.  The behavior tree is designed to be used with the PX4PosSetMove action server, which sends waypoints to the PX4 autopilot.  The PX4 autopilot then flies to the waypoints and sends feedback to the action server.  Currently, the example behavior tree is designed to fly to 3 waypoints and then land. The behavior tree is designed to be used with the AREAL Gazebo simulation, but it can be used with any PX4PosSetMove action server.

## Requirements
* ROS2 Foxy 
* BehaviorTree.CPP 
* areal_landing_px4_communication
* areal_landing_uav_interfaces
* px4_ros_com
* px4_msgs
* micro_ros_agent

<br>

Some good resources for micro_ros_agent are: https://github.com/Jaeyoung-Lim/px4-offboard and https://gist.github.com/julianoes/adbf76408663829cd9aed8d14c88fa29

<br>

## Running the Behavior Tree
1. Open up Gazebo.  This needs to be done from PX4-Autopilot.  Run the following command:
```
make px4_sitl gazebo_iris
```

In addition, you will need to modify the parameters of Gazebo for offboard control.  This should only need to be done one ever but if you ever have failsafes going off in Gazebo, then restart Gazebo and run these comands again.  Run these commands directly in the Gazebo terminal.
```
param set COM_RC_IN_MODE 1
param set COM_DISARM_PRFLT 30
param set COM_RCL_EXCEPT 4
```
</br>

2. Open up a new terminal and run the following command:
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

This is the bridge between ROS2 and the PX4 autopilot.</br></br>
3. Open up a new terminal.  Navigate to the folder '''/launch''' and run the following command:

```
ros2 launch waypoint_action_server.launch.py
```

</br></br>
4. If you have not yet built the behavior tree, open up a new terminal and run the following command:
```
colcon build --packages-select areal_new_bt
``` 
To start the behavior tree, run the following command:
```
ros2 run areal_new_bt new_bt
```
</br>

## Behavior Tree Package Changelog
**12/16/2022**</br>

*Editor: Ethan Tse*</br>
* Created basic README file.
* Removed mynode.cpp which was a basic hello world file.
* Got basic BehaviorTree.CPP tutorial tree working w/ ROS2.
* Split behavior_tree_sequencer.cpp into two files: behavior_tree_sequencer.hpp and main.cpp.
* Created tree.hpp where the behavior tree is defined. 

</br>

**12/15/2022**</br>
*Editor: Ethan Tse*</br>
* Created basic ROS2 node called "BehaviorTreeSequencer" which can subscribe to the PX4PosSetMove action server.

</br>

**01/11/2023**</br>
*Editor: Ethan Tse*</br>
* Implemented ROS2 nodes within Behaviortree.CPP nodes.  Created an example which flies to 3 points

</br>

**01/18/2023**</br>
*Editor: Ethan Tse*</br>
* Added basic blackboard to the behavior tree.  The blackboard is used to store the current waypoint index and the number of waypoints.

</br>

**01/23/2023**</br>
*Editor: Ethan Tse*</br>
* Cleaned code.

</br>

**03/23/2023**</br>
*Editor: Ethan Tse*</br>
* Rewrote to use a queue system and a fallbacknode.  This is a much cleaner implementation.


## Questions?
If you have any questions, please contact Adam Garlow at adamgarlow@icloud.com 
and Ethan Tse at etse8@gatech.edu 