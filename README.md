# drones

This repository contains all the code relevant to the drone exercises in the JdeRobot Robotics Academy. These packages are:
- `drone_wrapper`: Wrapper of the drone that extends its functionalities.
- `rqt_drone_teleop`: RQT GUI to teleoperate drones.

## Usage

The kinetic development has reached its End Of Life (EOL) but can still be installed using the command:

```bash
sudo apt-get install ros-kinetic-drone-wrapper ros-kinetic-rqt-drone-teleop
```
## ROS Distro Support

|         | Kinetic |
| ----- | ----- |
| Branch  | [`kinetic-devel`](https://github.com/JdeRobot/drones/tree/kinetic-devel) |
| Status  |  EOL |
| Version | [1.0.1](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=jderobot_drones) |

## ROS Buildfarm

|         |  Kinetic Source  |  Kinetic Debian |
| ----- | ----- | ----- |
| drone_wrapper | [![released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__drone_wrapper__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__drone_wrapper__ubuntu_xenial__source/) | [![released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__drone_wrapper__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__drone_wrapper__ubuntu_xenial_amd64__binary/) |
| rqt_drone_teleop | [![released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__rqt_drone_teleop__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__rqt_drone_teleop__ubuntu_xenial__source/) | [![released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__rqt_drone_teleop__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__rqt_drone_teleop__ubuntu_xenial_amd64__binary/) |

## Installation

An installation guide to Ubuntu 16.04.6 LTS (Xenial Xerus) can be found [here](https://github.com/JdeRobot/drones/blob/master/installation16.md).