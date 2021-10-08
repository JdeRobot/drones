# drones

This repository contains all the code relevant to the drone exercises in the JdeRobot Robotics Academy. The code is organized in three packages and one metapackage. These packages are:
- `jderobot_drones`: Metapackage, contains all the drone packages.
- `drone_wrapper`: Wrapper of the drone that extends its functionalities.
- `rqt_drone_teleop`: RQT GUI to teleoperate drones.
- `rqt_ground_robot_teleop`: RQT GUI to teleoperate ground robots.
- `drone_assets`: Assets needed in Academy exercises.
- `tello_driver`: Driver to control Tello drones with mavros and drone_wrapper. **Not included in metapackage**.

## Usage

### ROS Melodic

The packages provided here depend on the PX4 Firmware and SITL. Packages can be installed through:
```bash
sudo apt-get install ros-melodic-jderobot-drones
```

Notice that the previous command is equal to install each package separately:
```bash
sudo apt-get install ros-melodic-drone-assets ros-melodic-drone-wrapper ros-melodic-rqt-drone-teleop ros-melodic-rqt-ground-robot-teleop ros-melodic-tello-driver
```
If you just need one of the packages instead of all, you can install it separately.

## ROS Distro Support

|         | Melodic |
| ------- |:-------:|
| Branch  | [`melodic-devel`](https://github.com/JdeRobot/drones/tree/melodic-devel) |
| Status  | supported |
| Version | [1.3.8](http://repositories.ros.org/status_page/ros_melodic_default.html?q=jderobot_drones)<sup>1</sup> |

<sup>1</sup> _Melodic versions: 1.3.X_

## ROS Buildfarm

### Ubuntu

| `jderobot_drones` |  Melodic Bionic |
|:-------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__jderobot_drones__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__jderobot_drones__ubuntu_bionic__source/) |
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__jderobot_drones__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__jderobot_drones__ubuntu_bionic_amd64__binary/) |
| bin i386 | n/a |
| dev amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__jderobot_drones__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__jderobot_drones__ubuntu_bionic_amd64/) |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__jderobot_drones__ubuntu_bionic_arm64__binary)](http://build.ros.org/job/Mbin_ubv8_uBv8__jderobot_drones__ubuntu_bionic_arm64__binary/) |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubhf_uBhf__jderobot_drones__ubuntu_bionic_armhf__binary)](http://build.ros.org/job/Mbin_ubhf_uBhf__jderobot_drones__ubuntu_bionic_armhf__binary/) |


<details>
<summary><b>drone_assets</b></summary>
<br>
  
| `drone_assets` |  Melodic Bionic |
|:-------:|:-------------------:|
| source | [![Build Status](https://build.ros.org/buildStatus/icon?job=Msrc_uB__drone_assets__ubuntu_bionic__source)](https://build.ros.org/job/Msrc_uB__drone_assets__ubuntu_bionic__source/) | 
| bin amd64 | [![Build Status](https://build.ros.org/buildStatus/icon?job=Mbin_uB64__drone_assets__ubuntu_bionic_amd64__binary)](https://build.ros.org/job/Mbin_uB64__drone_assets__ubuntu_bionic_amd64__binary/) |
| bin i386 | n/a | 
| dev amd64 | n/a |
| bin arm64 | [![Build Status](https://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__drone_assets__ubuntu_bionic_arm64__binary)](https://build.ros.org/job/Mbin_ubv8_uBv8__drone_assets__ubuntu_bionic_arm64__binary/) |
| bin armhf |  [![Build Status](https://build.ros.org/buildStatus/icon?job=Mbin_ubhf_uBhf__drone_assets__ubuntu_bionic_armhf__binary)](https://build.ros.org/job/Mbin_ubhf_uBhf__drone_assets__ubuntu_bionic_armhf__binary/) |

</details>


<details>
<summary><b>drone_wrapper</b></summary>
<br>
  
| `drone_wrapper` |  Melodic Bionic |
|:-------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__drone_wrapper__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__drone_wrapper__ubuntu_bionic__source/) | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__drone_wrapper__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__drone_wrapper__ubuntu_bionic_amd64__binary/) |
| bin i386 | n/a | 
| dev amd64 | n/a |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__drone_wrapper__ubuntu_bionic_arm64__binary)](http://build.ros.org/job/Mbin_ubv8_uBv8__drone_wrapper__ubuntu_bionic_arm64__binary/) |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubhf_uBhf__drone_wrapper__ubuntu_bionic_armhf__binary)](http://build.ros.org/job/Mbin_ubhf_uBhf__drone_wrapper__ubuntu_bionic_armhf__binary/) |
  
</details>


<details>
<summary><b>rqt_drone_teleop</b></summary>
<br>
  
| `rqt_drone_teleop` |  Melodic Bionic |
|:-------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__rqt_drone_teleop__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__rqt_drone_teleop__ubuntu_bionic__source/) | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__rqt_drone_teleop__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__rqt_drone_teleop__ubuntu_bionic_amd64__binary/) | 
| bin i386 | n/a |
| dev amd64 | n/a |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__rqt_drone_teleop__ubuntu_bionic_arm64__binary)](http://build.ros.org/job/Mbin_ubv8_uBv8__rqt_drone_teleop__ubuntu_bionic_arm64__binary/) |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubhf_uBhf__rqt_drone_teleop__ubuntu_bionic_armhf__binary)](http://build.ros.org/job/Mbin_ubhf_uBhf__rqt_drone_teleop__ubuntu_bionic_armhf__binary/) |
  
</details>

<details>
<summary><b>rqt_ground_robot_teleop</b></summary>
<br>
  
| `rqt_ground_robot_teleop` |  Melodic Bionic |
|:-------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__rqt_ground_robot_teleop__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__rqt_ground_robot_teleop__ubuntu_bionic__source/) | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__rqt_ground_robot_teleop__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__rqt_ground_robot_teleop__ubuntu_bionic_amd64__binary/) | 
| bin i386 | n/a | 
| dev amd64 | n/a |
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__rqt_ground_robot_teleop__ubuntu_bionic_arm64__binary)](http://build.ros.org/job/Mbin_ubv8_uBv8__rqt_ground_robot_teleop__ubuntu_bionic_arm64__binary/) |
| bin armhf | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubhf_uBhf__rqt_ground_robot_teleop__ubuntu_bionic_armhf__binary)](http://build.ros.org/job/Mbin_ubhf_uBhf__rqt_ground_robot_teleop__ubuntu_bionic_armhf__binary/) |
  
</details>


<details>
<summary><b>tello_driver</b></summary>
<br>
  
| `tello_driver` |  Melodic Bionic |
|:-------:|:-------------------:|
| source | - | 
| bin amd64 | - |
| bin i386 | n/a | 
| dev amd64 | n/a |
| bin arm64 | - |
| bin armhf | - |

</details>

### Debian

| `jderobot_drones` |  Melodic Stretch   | 
|:-------:|:-------------------:|
| source | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_dS__jderobot_drones__debian_stretch__source)](http://build.ros.org/job/Msrc_dS__jderobot_drones__debian_stretch__source/) | 
| bin amd64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ds_dS64__jderobot_drones__debian_stretch_amd64__binary)](http://build.ros.org/job/Mbin_ds_dS64__jderobot_drones__debian_stretch_amd64__binary/) | 
| bin arm64 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_dsv8_dSv8__jderobot_drones__debian_stretch_arm64__binary)](http://build.ros.org/job/Mbin_dsv8_dSv8__jderobot_drones__debian_stretch_arm64__binary/) | 


<details>
<summary><b>drone_assets</b></summary>
<br>
  
| `drone_assets` |  Melodic Stretch   | 
|:-------:|:-------------------:|
| source | n/a | 
| bin amd64 | n/a | 
| bin arm64 | n/a | 
  
</details>


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

<details>
<summary><b>tello_driver</b></summary>
<br>
  
| `tello_driver` |  Melodic Stretch   | 
|:-------:|:-------------------:|
| source | - | 
| bin amd64 | - | 
| bin arm64 | - | 
  
</details>

## Installation

An installation guide to Ubuntu 18.04.3 LTS (Bionic Beaver) can be found [here](https://github.com/JdeRobot/drones/blob/master/installation18.md).