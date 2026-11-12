# hero_chassis_controller

## Overview

This is controller for the assignment (see Chinese [requirement](doc/requirement.md)) which assess
DynamicX reserve member.

**Keywords:** RoboMaster, ROS, ros_control, mecanum wheel

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Pengjie Xiao<br />
Affiliation: [GDUT DynamicX]()<br />
Maintainer: Pengjie Xiao, 2658167252@qq.com**

The hero_chassis_controller package has been tested under [ROS] Noetic on respective 20.04. This is
research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description_for_task](https://github.com/Bluefairy-ljy/rm_description_for_task.git)
- controller_interface
- hardware_interface
- pluginlib
- control_toolbox
- realtime_tools
- nav_msgs
- tf2, tf2_ros, tf2_geometry_msgs

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone git@github.com:xpj2007/hero_chassis_controller.git
    # git clone https://github.com/xpj2007/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build

## Usage

Run the simulation and controller with:

	roslaunch hero_chassis_controller run_simulation_and_controller.launch

Then unpause the simulation :

	rosservice call /gazebo/unpause_physics

Run the keyboard teleop in another terminal (publishes to `/cmd_vel`) :

	rosrun hero_chassis_controller key

## Config files

Config file config

* **controllers.yaml**  Params of hero_chassis_controller and joint_state_controller.

## Launch files

* **run_simulation_and_controller.launch:** Hero chassis only simulation and hero chassis controller

* **load_controller.launch:** Load controllers.yaml and start hero_chassis_controller and joint_state_controller

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/xpj2007/hero_chassis_controller/issues)
.

[ROS]: http://www.ros.org