# drones

This repository contains all the code relevant to the drone exercises in the JdeRobot Robotics Academy. The code is organized in three packages and one metapackage. These packages are:
- `jderobot_drones`: Metapackage, contains all the drone packages.
- `drone_wrapper`: Wrapper of the drone that extends its functionalities.
- `rqt_drone_teleop`: RQT GUI to teleoperate drones.
- `rqt_ground_robot_teleop`: RQT GUI to teleoperate ground robots.
- `drone_assets`: Assets needed in Academy exercises.

## Usage

### ROS Melodic

The packages provided here depend on the PX4 Firmware and SITL. Packages can be installed through:
```bash
sudo apt-get install ros-melodic-jderobot-drones
```

Notice that the previous command is equal to install each package separately:
```bash
sudo apt-get install ros-melodic-drone-wrapper ros-melodic-rqt-drone-teleop ros-melodic-rqt-ground-robot-teleop
```
If you just need one of the packages instead of all, you can install it separately.

### ROS Kinetic

The kinetic development has reached its End Of Life (EOL) but can still be installed using the command:

```bash
sudo apt-get install ros-kinetic-drone-wrapper ros-kinetic-rqt-drone-teleop
```

## ROS Distro Support

|         | Noetic | Melodic | Kinetic |
| ------- | ------ | ------- | ------- |
| Branch  | - | [`melodic-devel`](https://github.com/JdeRobot/drones/tree/melodic-devel) | [`1.0.1`](https://github.com/JdeRobot/drones/tree/3121c69db0901d5031450bbbd05a8aa4f3f3f341) |
| Status  | WIP | supported | EOL |
| Version | -<sup>1</sup> | [1.3.2](http://repositories.ros.org/status_page/ros_melodic_default.html?q=jderobot_drones)<sup>2</sup> | [1.0.1](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=jderobot_drones) |

<sup>1</sup> _Noetic versions: 1.4.X_ --
<sup>2</sup> _Melodic versions: 1.3.X_

## ROS Buildfarm

### Ubuntu

| `jderobot_drones` |  Melodic Bionic |  Kinetic Xenial  |
|:-------:|:-------------------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__jderobot_drones__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__jderobot_drones__ubuntu_bionic__source/) | n/a | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__jderobot_drones__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__jderobot_drones__ubuntu_bionic_amd64__binary/) | n/a | 
| bin i386 | n/a | n/a | 
| dev amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__jderobot_drones__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__jderobot_drones__ubuntu_bionic_amd64/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__jderobot_drones__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdev__jderobot_drones__ubuntu_xenial_amd64/) |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__jderobot_drones__ubuntu_bionic_arm64__binary)](http://build.ros.org/job/Mbin_ubv8_uBv8__jderobot_drones__ubuntu_bionic_arm64__binary/) | n/a |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubhf_uBhf__jderobot_drones__ubuntu_bionic_armhf__binary)](http://build.ros.org/job/Mbin_ubhf_uBhf__jderobot_drones__ubuntu_bionic_armhf__binary/) | n/a |

<details>
<summary><b>drone_wrapper</b></summary>
<br>
  
| `drone_wrapper` |  Melodic Bionic |  Kinetic Xenial  |
|:-------:|:-------------------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__drone_wrapper__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__drone_wrapper__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__drone_wrapper__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__drone_wrapper__ubuntu_xenial__source/) | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__drone_wrapper__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__drone_wrapper__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__drone_wrapper__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__drone_wrapper__ubuntu_xenial_amd64__binary/) | 
| bin i386 | n/a | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX32__drone_wrapper__ubuntu_xenial_i386__binary)](http://build.ros.org/job/Kbin_uX32__drone_wrapper__ubuntu_xenial_i386__binary/) | 
| dev amd64 | n/a | n/a |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__drone_wrapper__ubuntu_bionic_arm64__binary)](http://build.ros.org/job/Mbin_ubv8_uBv8__drone_wrapper__ubuntu_bionic_arm64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uxv8_uXv8__drone_wrapper__ubuntu_xenial_arm64__binary)](http://build.ros.org/job/Kbin_uxv8_uXv8__drone_wrapper__ubuntu_xenial_arm64__binary/) |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubhf_uBhf__drone_wrapper__ubuntu_bionic_armhf__binary)](http://build.ros.org/job/Mbin_ubhf_uBhf__drone_wrapper__ubuntu_bionic_armhf__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uxhf_uXhf__drone_wrapper__ubuntu_xenial_armhf__binary)](http://build.ros.org/job/Kbin_uxhf_uXhf__drone_wrapper__ubuntu_xenial_armhf__binary/) |
  
</details>


<details>
<summary><b>rqt_drone_teleop</b></summary>
<br>
  
| `rqt_drone_teleop` |  Melodic Bionic |  Kinetic Xenial  |
|:-------:|:-------------------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__rqt_drone_teleop__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__rqt_drone_teleop__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__rqt_drone_teleop__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__rqt_drone_teleop__ubuntu_xenial__source/) | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__rqt_drone_teleop__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__rqt_drone_teleop__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__rqt_drone_teleop__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__rqt_drone_teleop__ubuntu_xenial_amd64__binary/) | 
| bin i386 | n/a | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX32__rqt_drone_teleop__ubuntu_xenial_i386__binary)](http://build.ros.org/job/Kbin_uX32__rqt_drone_teleop__ubuntu_xenial_i386__binary/) | 
| dev amd64 | n/a | n/a |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__rqt_drone_teleop__ubuntu_bionic_arm64__binary)](http://build.ros.org/job/Mbin_ubv8_uBv8__rqt_drone_teleop__ubuntu_bionic_arm64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uxv8_uXv8__rqt_drone_teleop__ubuntu_xenial_arm64__binary)](http://build.ros.org/job/Kbin_uxv8_uXv8__rqt_drone_teleop__ubuntu_xenial_arm64__binary/) |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubhf_uBhf__rqt_drone_teleop__ubuntu_bionic_armhf__binary)](http://build.ros.org/job/Mbin_ubhf_uBhf__rqt_drone_teleop__ubuntu_bionic_armhf__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uxhf_uXhf__rqt_drone_teleop__ubuntu_xenial_armhf__binary)](http://build.ros.org/job/Kbin_uxhf_uXhf__rqt_drone_teleop__ubuntu_xenial_armhf__binary/) |
  
</details>

<details>
<summary><b>rqt_ground_robot_teleop</b></summary>
<br>
  
| `rqt_ground_robot_teleop` |  Melodic Bionic |  Kinetic Xenial  |
|:-------:|:-------------------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__rqt_ground_robot_teleop__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__rqt_ground_robot_teleop__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__rqt_ground_robot_teleop__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__rqt_ground_robot_teleop__ubuntu_xenial__source/) | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__rqt_ground_robot_teleop__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__rqt_ground_robot_teleop__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__rqt_ground_robot_teleop__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__rqt_ground_robot_teleop__ubuntu_xenial_amd64__binary/) | 
| bin i386 | n/a | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX32__rqt_ground_robot_teleop__ubuntu_xenial_i386__binary)](http://build.ros.org/job/Kbin_uX32__rqt_ground_robot_teleop__ubuntu_xenial_i386__binary/) | 
| dev amd64 | n/a | n/a |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__rqt_ground_robot_teleop__ubuntu_bionic_arm64__binary)](http://build.ros.org/job/Mbin_ubv8_uBv8__rqt_ground_robot_teleop__ubuntu_bionic_arm64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uxv8_uXv8__rqt_ground_robot_teleop__ubuntu_xenial_arm64__binary)](http://build.ros.org/job/Kbin_uxv8_uXv8__rqt_ground_robot_teleop__ubuntu_xenial_arm64__binary/) |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubhf_uBhf__rqt_ground_robot_teleop__ubuntu_bionic_armhf__binary)](http://build.ros.org/job/Mbin_ubhf_uBhf__rqt_ground_robot_teleop__ubuntu_bionic_armhf__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uxhf_uXhf__rqt_ground_robot_teleop__ubuntu_xenial_armhf__binary)](http://build.ros.org/job/Kbin_uxhf_uXhf__rqt_ground_robot_teleop__ubuntu_xenial_armhf__binary/) |
  
</details>

### Debian

| `jderobot_drones` |  Melodic Stretch   | 
|:-------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_dS__jderobot_drones__debian_stretch__source)](http://build.ros.org/job/Msrc_dS__jderobot_drones__debian_stretch__source/) | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ds_dS64__jderobot_drones__debian_stretch_amd64__binary)](http://build.ros.org/job/Mbin_ds_dS64__jderobot_drones__debian_stretch_amd64__binary/) | 
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_dsv8_dSv8__jderobot_drones__debian_stretch_arm64__binary)](http://build.ros.org/job/Mbin_dsv8_dSv8__jderobot_drones__debian_stretch_arm64__binary/) | 

<details>
<summary><b>drone_wrapper</b></summary>
<br>
  
| `drone_wrapper` |  Melodic Stretch   | 
|:-------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_dS__drone_wrapper__debian_stretch__source)](http://build.ros.org/job/Msrc_dS__drone_wrapper__debian_stretch__source/) | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ds_dS64__drone_wrapper__debian_stretch_amd64__binary)](http://build.ros.org/job/Mbin_ds_dS64__drone_wrapper__debian_stretch_amd64__binary/) | 
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_dsv8_dSv8__drone_wrapper__debian_stretch_arm64__binary)](http://build.ros.org/job/Mbin_dsv8_dSv8__drone_wrapper__debian_stretch_arm64__binary/) | 
  
</details>

<details>
<summary><b>rqt_drone_teleop</b></summary>
<br>
  
| `rqt_drone_teleop` |  Melodic Stretch   | 
|:-------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_dS__rqt_drone_teleop__debian_stretch__source)](http://build.ros.org/job/Msrc_dS__rqt_drone_teleop__debian_stretch__source/) | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ds_dS64__rqt_drone_teleop__debian_stretch_amd64__binary)](http://build.ros.org/job/Mbin_ds_dS64__rqt_drone_teleop__debian_stretch_amd64__binary/) | 
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_dsv8_dSv8__rqt_drone_teleop__debian_stretch_arm64__binary)](http://build.ros.org/job/Mbin_dsv8_dSv8__rqt_drone_teleop__debian_stretch_arm64__binary/) | 
  
</details>

<details>
<summary><b>rqt_ground_robot_teleop</b></summary>
<br>
  
| `rqt_ground_robot_teleop` |  Melodic Stretch   | 
|:-------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_dS__rqt_ground_robot_teleop__debian_stretch__source)](http://build.ros.org/job/Msrc_dS__rqt_ground_robot_teleop__debian_stretch__source/) | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ds_dS64__rqt_ground_robot_teleop__debian_stretch_amd64__binary)](http://build.ros.org/job/Mbin_ds_dS64__rqt_ground_robot_teleop__debian_stretch_amd64__binary/) | 
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_dsv8_dSv8__rqt_ground_robot_teleop__debian_stretch_arm64__binary)](http://build.ros.org/job/Mbin_dsv8_dSv8__rqt_ground_robot_teleop__debian_stretch_arm64__binary/) | 
  
</details>

## Installation

An installation guide to Ubuntu 18.04.3 LTS (Bionic Beaver) can be found [here](https://github.com/JdeRobot/drones/blob/master/installation18.md).

An installation guide to Ubuntu 16.04.6 LTS (Xenial Xerus) can be found [here](https://github.com/JdeRobot/drones/blob/master/installation16.md).
