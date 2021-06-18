# drones

This repository contains all the code relevant to the drone exercises in the JdeRobot Robotics Academy. The code is organized in five packages and one metapackage. These packages are:
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
| Branch  | [`noetic-devel`](https://github.com/JdeRobot/drones/tree/noetic-devel) | [`melodic-devel`](https://github.com/JdeRobot/drones/tree/melodic-devel) | [`kinetic-devel`](https://github.com/JdeRobot/drones/tree/kinetic-devel)
| Status  | supported | supported | EOL |
| Version | [1.4.0](http://repositories.ros.org/status_page/ros_noetic_default.html?q=jderobot_drones)<sup>1</sup> | [1.3.8](http://repositories.ros.org/status_page/ros_melodic_default.html?q=jderobot_drones)<sup>2</sup> | [1.0.1](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=jderobot_drones)

<sup>1</sup> _Noetic versions: 1.4.X_ --
<sup>2</sup> _Melodic versions: 1.3.X_
