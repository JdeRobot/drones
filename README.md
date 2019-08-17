# drones

This repository contains all the code relevant to the drone exercises in the JdeRobot Robotics Academy.

## Usage

The packages provided here depend on the PX4 Firmware and SITL. We plan to provide this package as a deb file installable through `apt-get`, however, at the moment, the best method for the installation is to run the bash script to setup the dependancies and the catkin_ws and put the drone_wrapper package and rqt_follow_road package in the src.

## ROS Distro Support

|         | Kinetic |
| ----- | ----- |
| Branch  | [`master`](https://github.com/JdeRobot/drones/tree/master/) |
| Status  |  supported |
| Version | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=jderobot_drones) |

## ROS Buildfarm

|         |  Kinetic Source  |  Kinetic Debian |
| ----- | ----- | ----- |
| drone_wrapper | [![released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__drone_wrapper__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__drone_wrapper__ubuntu_xenial__source/) | [![released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__drone_wrapper__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__drone_wrapper__ubuntu_xenial_amd64__binary/) |
| rqt_drone_teleop | [![released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__rqt_drone_teleop__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__rqt_drone_teleop__ubuntu_xenial__source/) | [![released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__rqt_drone_teleop__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__rqt_drone_teleop__ubuntu_xenial_amd64__binary/) |
