# drones

This repository contains all the code relevant to the drone exercises in the JdeRobot Robotics Academy. The code is organized in three packages and one metapackage. These packages are:
- `jderobot_drone`: Metapackage, contains all the drone packages.
- `drone_wrapper`: Wrapper of the drone that extends its functionalities.
- `rqt_drone_teleop`: RQT GUI to teleoperate drones.
- `rqt_ground_robot_teleop`: RQT GUI to teleoperate ground robots.

## Usage

The packages provided here depend on the PX4 Firmware and SITL. Packages can be installed through:
```bash
sudo apt-get install ros-melodic-jderobot-drones
```

Notice that the previous command is equal to install each package separately:
```bash
sudo apt-get install ros-melodic-drone-wrapper ros-melodic-rqt-drone-teleop ros-melodic-rqt-ground-robot-teleop
```
If you just need one of the packages instead of all, you can install it separately.


## ROS Distro Support

|         | Kinetic | Melodic |
| ----- | ----- | ----- |
| Branch  | [`1.0.1`](https://github.com/JdeRobot/drones/tree/3121c69db0901d5031450bbbd05a8aa4f3f3f341) | [`master`](https://github.com/JdeRobot/drones/tree/master/) |
| Status  |  obsolete | supported |
| Version | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=jderobot_drones) | [version](http://repositories.ros.org/status_page/ros_melodic_default.html?q=jderobot_drones)

## ROS Buildfarm

|         |  Kinetic Source  |  Kinetic Debian |
| ----- | ----- | ----- |
| drone_wrapper | [![released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__drone_wrapper__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__drone_wrapper__ubuntu_xenial__source/) | [![released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__drone_wrapper__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__drone_wrapper__ubuntu_xenial_amd64__binary/) |
| rqt_drone_teleop | [![released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__rqt_drone_teleop__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__rqt_drone_teleop__ubuntu_xenial__source/) | [![released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__rqt_drone_teleop__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__rqt_drone_teleop__ubuntu_xenial_amd64__binary/) |


|         |  Melodic Source  |  Melodic Debian |
| ----- | ----- | ----- |
| jderobot_drones | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__jderobot_drones__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__jderobot_drones__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__jderobot_drones__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__jderobot_drones__ubuntu_bionic_amd64__binary/) | 
| drone_wrapper | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__drone_wrapper__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__drone_wrapper__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__drone_wrapper__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__drone_wrapper__ubuntu_bionic_amd64__binary/) |
| rqt_drone_teleop | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__rqt_drone_teleop__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__rqt_drone_teleop__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__rqt_drone_teleop__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__rqt_drone_teleop__ubuntu_bionic_amd64__binary/) |
| rqt_ground_robot_teleop | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__rqt_ground_robot_teleop__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__rqt_ground_robot_teleop__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__rqt_ground_robot_teleop__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__rqt_ground_robot_teleop__ubuntu_bionic_amd64__binary/) |

## Installation

An installation guide to Ubuntu 18.04.3 LTS (Bionic Beaver) can be found [here](https://github.com/JdeRobot/drones/blob/master/installation18.md).

An installation guide to Ubuntu 16.04.6 LTS (Xenial Xerus) can be found [here](https://github.com/JdeRobot/drones/blob/master/installation16.md).
