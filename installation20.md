# Installation Guide
This file explains how to install all the software infrastructure related to JdeRobot Academy drones on Ubuntu 20.04.2.0 (Focal Fossa).

## Step 1: ROS Noetic, gazebo11, JdeRobot Academy.
- Install ROS Noetic (ros-noetic-desktop-full): [Link](https://wiki.ros.org/noetic/Installation/Ubuntu).
- Install gazebo11 (alternative installation): [Link](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
- Clone JdeRobot Academy (git required): 
```bash
git clone https://github.com/JdeRobot/Academy.git
```

**_Note:_** At this point you should try `roscore &` and `rosrun gazebo_ros gazebo` to test if the basic infrastructure works.

## Step 2: MavROS and PX4.
### 2.1. Install MAVROS (v1.7.0)
```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
```

### 2.2. Install PX4 dependencies 

1. Download and install Geographiclib
```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm ./install_geographiclib_datasets.sh  # optional
```

2. Install common dependencies
```bash
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool \
    python-pip python-dev -y
```

3. Install xxd
```bash
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y
```

4. Install python-is-python3 for Ubuntu20.04+
```bash
sudo apt install python-is-python3
```

### 2.4. PX4 source installation

1. Get PX4 source (v1.11.3)
```bash
mkdir ~/repos && cd repos
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot && git checkout -b v1.11.3
```

2. Run PX4 installation script
```bash
cd Tools/setup/
bash ubuntu.sh --no-nuttx --no-sim-tools
```

3. Install gstreamer
```bash
sudo apt install libgstreamer1.0-dev
sudo apt install gstreamer1.0-plugins-bad
```

4. Build PX4
```bash
cd ~/repos/PX4-Autopilot
DONT_RUN=1 make px4_sitl gazebo
```

5. Export environment variables
```bash
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/repos/PX4-Autopilot/build/px4_sitl_default/build_gazebo' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/repos/PX4-Autopilot/Tools/sitl_gazebo/models' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/repos/PX4-Autopilot/build/px4_sitl_default/build_gazebo' >> ~/.bashrc
    
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/repos/PX4-Autopilot' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/repos/PX4-Autopilot/Tools/sitl_gazebo' >> ~/.bashrc
    
source ~/.bashrc
```

6. Try PX4 (optional)
```bash
roslaunch px4 mavros_posix_sitl.launch
pxh> commander arm # when launching finishes
```

## Step 3: JdeRobot-drones
### A. Binary installation
```bash
sudo apt-get install ros-noetic-jderobot-drones
```

### B. Source installation
1. Install catkin tools
```bash
sudo apt install python3-catkin-tools python3-osrf-pycommon
```

2. Set up catkin workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
echo 'export ROS_WORKSPACE=~/catkin_ws' >> ~/.bashrc # points roscd dir
source ~/.bashrc
```

3. Get jderobot-drones repository that contains ros pkgs
```bash
cd ~/repos
git clone https://github.com/JdeRobot/drones.git
cd drones
git checkout -b noetic-devel
```

4. Link drone source to catkin_ws
```bash
roscd && cd src
ln -s ~/repos/drones/drone_wrapper .
ln -s ~/repos/drones/drone_assets .
ln -s ~/repos/drones/rqt_drone_teleop .
```

5. Update ros dependencies
```bash
roscd
rosdep init  # needs to be called ONLY once after installation. sudo might be required
rosdep update # do not run with sudo
#rosdep install --from-paths . --ignore-src --rosdistro noetic -y  #sudo might be required
```

6. Build 
```bash
 roscd && catkin build
```

7. Export environment variables
```bash
roscd
echo 'source '$PWD'/devel/setup.bash' >> ~/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/usr/share/gazebo-11' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/repos/drones/drone_assets/models' >> ~/.bashrc
source ~/.bashrc
```

## 4. Try it!
Test an exercise. E.g. At RoboticsAcademy/exercises/position_control `roslaunch position_control.launch`.

***

# Troubleshooting
If you had troubles during installation or execution, you may be lucky and find a solution here.

## My drone does not respond
If your drone does nothing or does not do what I expect it to do, you probably have errors in the code. Some errors like indentation errors appear at the start of the execution, and not after pushing the `play code` button.

## Bad environment variable export
Traceback:
```bash
root@9a306b9595fe:/RoboticsAcademy/exercises/drone_cat_mouse# roslaunch drone_cat_mouse.launch 
... logging to /root/.ros/log/5e199352-66ef-11eb-bf4f-0242ac110002/roslaunch-9a306b9595fe-133.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

Resource not found: px4
ROS path [0]=/opt/ros/noetic/share/ros
ROS path [1]=/opt/ros/noetic/share
The traceback for the exception was written to the log file
```

Make sure to export correctly all the envvars. You can use `rospack find <pkg>` to check if a ros package is well installed.

## Problem with OpenCV
If you have seen something like: `rqt_gui process has died`, you should check that your opencv version is correct.

Make sure that OpenCv version is 3.2.0. Besides, opencv have to be installed with `sudo apt-get install python-opencv`. Do not install opencv with pip.

You can check opencv version with:

```bash
$ apt list python-opencv -a
Listando... Hecho
python-opencv/bionic-updates,bionic-security,now 3.2.0+dfsg-4ubuntu0.1 amd64 [instalado]
python-opencv/bionic 3.2.0+dfsg-4build2 amd64

$ python2
Python 2.7.17 (default, Feb 25 2021, 14:02:55) 
[GCC 7.5.0] on linux2
Type "help", "copyright", "credits" or "license" for more information.
>>> import cv2
>>> print cv2.__version__
3.2.0
```

## Problem with mavros_msgs
Traceback:
```bash
[ERROR] [1611940181.282428418, 2.588000000]: Client [/cat/rqt_gui] wants topic /cat/mavros/state to have 
datatype/md5sum [mavros_msgs/State/ce783f756cab1193cb71ba9e90fece50], but our version has 
[mavros_msgs/State/65cd0a9fff993b062b91e354554ec7e9]. Dropping connection.
```

Make sure that `mavros`, `mavros_msgs` and `mavros_extras` are up to date. Their dependencies must also be up to date. Some ros packages like `ros-noetic-std-msgs` might be out to date.

The easiest solution is to upgrade all the apt packages in the computer:
```bash
sudo apt update
sudo apt upgrade
```

## Problem with PX4
PX4 version before v1.11 were installed through the catkin workspace. Current versions are installed separately. Make sure to install PX4 outside your catkin_ws folder!

## roslaunch didn't find some models
Check your environment variables, primarily `GAZEBO_MODEL_PATH`. This envvar have to contain a path to `drone_assets`, among others.

If you have installed jderobot-drones from source, make sure that your repo is up to date.

## Problem with gazebo (_ignite-tool_, _ignite-fuel_, _ignite-math_, _ignite-*_...)
Make sure to be updated and upgraded.
```bash
sudo apt update
sudo apt upgrade
```

## Problem with catkin_ws dependencies
As it had been explained, make sure to have updated ROS dependencies.
```bash
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro noetic
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
```

## Failure to install Geographiclib dataset
Links may be broken. Check this [issue](https://github.com/mavlink/mavros/issues/963).

## qfi package not found
Please follow qfi installation instructions [here](https://github.com/JdeRobot/ThirdParty/tree/master/qflightinstruments).
Make sure to run last instruction with superuser priviledges (´sudo make install´).

## During qfi installation, _unknown module in QT: svg_
Check [this](https://stackoverflow.com/questions/21098805/unknown-modules-in-qt-svg) out.

