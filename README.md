# drones

This repository contains all the code relevant to the drone exercises in the JdeRobot Robotics Academy. The code is organized in three packages and one metapackage. These packages are:
- `jderobot_drones`: Metapackage, contains all the drone packages.
- `drone_wrapper`: Wrapper of the drone that extends its functionalities.
- `rqt_drone_teleop`: RQT GUI to teleoperate drones.
- `rqt_ground_robot_teleop`: RQT GUI to teleoperate ground robots.
- `drone_assets`: Assets needed in Academy exercises.
- `tello_driver`: Driver to control Tello drones with mavros and drone_wrapper. **Not included in metapackage**.

## Usage

### ROS Noetic

The packages provided here depend on the PX4 Firmware and SITL. Packages can be installed through:
```bash
sudo apt-get install ros-noetic-jderobot-drones
```

Notice that the previous command is equal to install each package separately:
```bash
sudo apt-get install ros-noetic-drone-wrapper ros-noetic-rqt-drone-teleop ros-noetic-rqt-ground-robot-teleop
```
If you just need one of the packages instead of all, you can install it separately.

## ROS Distro Support

|         | Noetic |
| ------- |:------:|
| Branch  | [`noetic-devel`](https://github.com/JdeRobot/drones/tree/noetic-devel) |
| Status  | supported |
| Version | [1.4.0](http://repositories.ros.org/status_page/ros_noetic_default.html?q=jderobot_drones)<sup>1</sup> |

<sup>1</sup> _Noetic versions: 1.4.X_

## ROS Buildfarm

### Ubuntu

| `jderobot_drones` |  Noetic Focal |
|:-----------------:|:-------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__jderobot_drones__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__jderobot_drones__ubuntu_focal__source/) |
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__jderobot_drones__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__jderobot_drones__ubuntu_focal__source/) |
| bin i386 | n/a |
| dev amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndev__jderobot_drones__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__jderobot_drones__ubuntu_focal_amd64/) |
| bin arm64 | [![Build Status](https://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__jderobot_drones__ubuntu_focal_arm64__binary)](https://build.ros.org/job/Nbin_ufv8_uFv8__jderobot_drones__ubuntu_focal_arm64__binary/) |
| bin armhf | [![Build Status](https://build.ros.org/buildStatus/icon?job=Nbin_ufhf_uFhf__jderobot_drones__ubuntu_focal_armhf__binary)](https://build.ros.org/job/Nbin_ufhf_uFhf__jderobot_drones__ubuntu_focal_armhf__binary/) |

<details>
<summary><b>drone_assets</b></summary>
<br>
  
| `drone_assets` |  Noetic Focal |
|:---------------:|:-------------:|
| source | [![Build Status](https://build.ros.org/buildStatus/icon?job=Nsrc_uF__drone_assets__ubuntu_focal__source)](https://build.ros.org/job/Nsrc_uF__drone_assets__ubuntu_focal__source/) |
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__drone_assets__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__drone_assets__ubuntu_focal_amd64__binary/) |
| bin i386 | n/a |
| dev amd64 | n/a |
| bin arm64 |[![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__drone_assets__ubuntu_focal_arm64__binary)](http://build.ros.org/job/Nbin_ufv8_uFv8__drone_assets__ubuntu_focal_arm64__binary/) |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ufhf_uFhf__drone_assets__ubuntu_focal_armhf__binary)](http://build.ros.org/job/Nbin_ufhf_uFhf__drone_assets__ubuntu_focal_armhf__binary/) |
</details>

<details>
<summary><b>drone_wrapper</b></summary>
<br>
  
| `drone_wrapper` |  Noetic Focal |
|:---------------:|:-------------:|
| source | [![Build Status](https://build.ros.org/buildStatus/icon?job=Nsrc_uF__drone_wrapper__ubuntu_focal__source)](https://build.ros.org/job/Nsrc_uF__drone_wrapper__ubuntu_focal__source/) |
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__drone_wrapper__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__drone_wrapper__ubuntu_focal_amd64__binary/) |
| bin i386 | n/a | 
| dev amd64 | n/a |
| bin arm64 |[![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__drone_wrapper__ubuntu_focal_arm64__binary)](http://build.ros.org/job/Nbin_ufv8_uFv8__drone_wrapper__ubuntu_focal_arm64__binary/) |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ufhf_uFhf__drone_wrapper__ubuntu_focal_armhf__binary)](http://build.ros.org/job/Nbin_ufhf_uFhf__drone_wrapper__ubuntu_focal_armhf__binary/) |
</details>

<details>
<summary><b>rqt_drone_teleop</b></summary>
<br>
  
| `rqt_drone_teleop` |  Noetic Focal |
|:------------------:|:-------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__rqt_drone_teleop__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__rqt_drone_teleop__ubuntu_focal__source/) |
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__rqt_drone_teleop__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__rqt_drone_teleop__ubuntu_focal_amd64__binary/) | 
| bin i386 | n/a |
| dev amd64 | n/a |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__rqt_drone_teleop__ubuntu_focal_arm64__binary)](http://build.ros.org/job/Nbin_ufv8_uFv8__rqt_drone_teleop__ubuntu_focal_arm64__binary/) |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ufhf_uFhf__rqt_drone_teleop__ubuntu_focal_armhf__binary)](http://build.ros.org/job/Nbin_ufhf_uFhf__rqt_drone_teleop__ubuntu_focal_armhf__binary/) |
</details>

<details>
<summary><b>rqt_ground_robot_teleop</b></summary>
<br>
  
| `rqt_ground_robot_teleop` |  Noetic Focal |
|:-------------------------:|:-------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__rqt_ground_robot_teleop__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__rqt_ground_robot_teleop__ubuntu_focal__source/) |
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__rqt_ground_robot_teleop__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__rqt_ground_robot_teleop__ubuntu_focal_amd64__binary/) |
| bin i386 | n/a |
| dev amd64 | n/a |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__rqt_ground_robot_teleop__ubuntu_focal_arm64__binary)](http://build.ros.org/job/Nbin_ufv8_uFv8__rqt_ground_robot_teleop__ubuntu_focal_arm64__binary/) |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ufhf_uFhf__rqt_ground_robot_teleop__ubuntu_focal_armhf__binary)](http://build.ros.org/job/Nbin_ufhf_uFhf__rqt_ground_robot_teleop__ubuntu_focal_armhf__binary/) |
</details>

### Debian

| `jderobot_drones` | Noetic Stretch | 
|:-----------------:|:--------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_dS__jderobot_drones__debian_stretch__source)](http://build.ros.org/job/Nsrc_dS__jderobot_drones__debian_stretch__source/) |
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ds_dS64__jderobot_drones__debian_stretch_amd64__binary)](http://build.ros.org/job/Nbin_ds_dS64__jderobot_drones__debian_stretch_amd64__binary/) | 
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_dsv8_dSv8__jderobot_drones__debian_stretch_arm64__binary)](http://build.ros.org/job/Nbin_dsv8_dSv8__jderobot_drones__debian_stretch_arm64__binary/) |

<details>
<summary><b>drone_assets</b></summary>
<br>
  
| `drone_assets` | Noetic Stretch | 
|:--------------:|:--------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_dS__drone_assets__debian_stretch__source)](http://build.ros.org/job/Nsrc_dS__drone_assets__debian_stretch__source/) |
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ds_dS64__drone_assets__debian_stretch_amd64__binary)](http://build.ros.org/job/Nbin_ds_dS64__drone_assets__debian_stretch_amd64__binary/) |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_dsv8_dSv8__drone_assets__debian_stretch_arm64__binary)](http://build.ros.org/job/Nbin_dsv8_dSv8__drone_assets__debian_stretch_arm64__binary/) |
  
</details>

<details>
<summary><b>drone_wrapper</b></summary>
<br>
  
| `drone_wrapper` | Noetic Stretch |
|:---------------:|:--------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_dS__drone_wrapper__debian_stretch__source)](http://build.ros.org/job/Nsrc_dS__drone_wrapper__debian_stretch__source/) |
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ds_dS64__drone_wrapper__debian_stretch_amd64__binary)](http://build.ros.org/job/Nbin_ds_dS64__drone_wrapper__debian_stretch_amd64__binary/) |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_dsv8_dSv8__drone_wrapper__debian_stretch_arm64__binary)](http://build.ros.org/job/Nbin_dsv8_dSv8__drone_wrapper__debian_stretch_arm64__binary/) |
  
</details>

<details>
<summary><b>rqt_drone_teleop</b></summary>
<br>
  
| `rqt_drone_teleop` | Noetic Stretch |
|:------------------:|:--------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_dS__rqt_drone_teleop__debian_stretch__source)](http://build.ros.org/job/Nsrc_dS__rqt_drone_teleop__debian_stretch__source/) |
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ds_dS64__rqt_drone_teleop__debian_stretch_amd64__binary)](http://build.ros.org/job/Nbin_ds_dS64__rqt_drone_teleop__debian_stretch_amd64__binary/) |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_dsv8_dSv8__rqt_drone_teleop__debian_stretch_arm64__binary)](http://build.ros.org/job/Nbin_dsv8_dSv8__rqt_drone_teleop__debian_stretch_arm64__binary/) |
  
</details>

<details>
<summary><b>rqt_ground_robot_teleop</b></summary>
<br>
  
| `rqt_ground_robot_teleop` | Noetic Stretch |
|:-------------------------:|:--------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_dS__rqt_ground_robot_teleop__debian_stretch__source)](http://build.ros.org/job/Nsrc_dS__rqt_ground_robot_teleop__debian_stretch__source/) |
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ds_dS64__rqt_ground_robot_teleop__debian_stretch_amd64__binary)](http://build.ros.org/job/Nbin_ds_dS64__rqt_ground_robot_teleop__debian_stretch_amd64__binary/) |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_dsv8_dSv8__rqt_ground_robot_teleop__debian_stretch_arm64__binary)](http://build.ros.org/job/Nbin_dsv8_dSv8__rqt_ground_robot_teleop__debian_stretch_arm64__binary/) |
  
</details>

## Installation

An installation guide to Ubuntu 20.04.2 LTS (Focal Fosaa) can be found [here](https://github.com/JdeRobot/drones/blob/noetic-devel/installation20.md).
