# Installation Guide
This file explains how to install all the software infrastructure related to JdeRobot Academy drones on Ubuntu 18.04.03 (Bionic Beaver).

## Step 1: ROS Melodic, gazebo9, JdeRobot assets.
- Install ROS Melodic (ros-melodic-desktop-full): [Link](https://wiki.ros.org/melodic/Installation/Ubuntu).
- Install gazebo9 (alternative installation): [Link](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
- Add JdeRobot sources, set the key and install JdeRobot assets:
```
sudo sh -c 'cat<<EOF>/etc/apt/sources.list.d/jderobot.list
# for ubuntu 16.04 LTS (64 bit)

deb [arch=amd64] http://wiki.jderobot.org/apt bionic main
EOF'
```
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv 24E521A4
```
```
sudo apt update
sudo apt install jderobot-gazebo-assets
```
- Clone JdeRobot Academy (git needed): 
```
git clone https://github.com/JdeRobot/Academy.git
```

**_Note:_** At this point you should try `roscore &` and `rosrun gazebo_ros gazebo` to test if the basic infrastructure works.

## Step 2: Dependencies needed for the exercices.
### 2.1. Install MAVROS and Geographiclib
```
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

### 2.2. Remove modemmanager
```
sudo apt-get remove modemmanager
```

### 2.3. Install common dependencies
```
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool -y
```

### 2.4. Install xxd
```
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y
```

### 2.5. Required python packages
```
sudo apt-get install python-argparse \
    python-empy python-toml python-numpy python-yaml \
    python-dev python-pip -y
sudo -H pip install --upgrade pip 
sudo -H pip install pandas jinja2 pyserial cerberus
sudo -H pip install pyulog
```

### 2.6. Install ninja
```
sudo apt-get install ninja-build -y
```

### 2.7. Get FastRTPS and FastCDR
```
wget https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-7-1/eprosima_fastrtps-1-7-1-linux-tar-gz -O eprosima_fastrtps-1-7-1-linux.tar.gz
tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz eProsima_FastRTPS-1.7.1-Linux/
tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz requiredcomponents
tar -xzf requiredcomponents/eProsima_FastCDR-1.0.8-Linux.tar.gz
```

### 2.8. Build FastRTPS and FastCDR
```
(cd eProsima_FastCDR-1.0.8-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
(cd eProsima_FastRTPS-1.7.1-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
rm -rf requiredcomponents eprosima_fastrtps-1-7-1-linux.tar.gz
```

## 3. Set up catkin workspace
### 3.1. Install catkin tools
```
sudo apt-get install python-catkin-tools
```

### 3.2. Set up workspace
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/PX4/Firmware.git
cd Firmware
git submodule update --init --recursive
cd ..
ln -s Firmware/Tools/sitl_gazebo mavlink_sitl_gazebo
cd ..
```

### 3.3. Update ROS dependencies
```
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro melodic
rosdep install --from-paths . --ignore-src --rosdistro melodic -y
```

### 3.4. Build catkin (make sure to be at /catkin_ws)
```
catkin build
```

### 3.5. Export environment variables
Provisional value for GAZEBO_MODEL_PATH variable.
```
echo 'source '$PWD'/devel/setup.bash' >> ~/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/usr/share/gazebo-9' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/opt/ros/melodic/share/jderobot_assets/models:~/catkin_ws/src/jderobot_assets/models' >> ~/.bashrc

source ~/.bashrc
```

**_Note:_** Test installation using: `roslaunch px4 mavros_posix_sitl.launch`

## 4. Install the lastest ROS packages manually (soon it will be faster and easier)
- Download the following packages [drone-wrapper, rqt-drone-teleop](https://github.com/JdeRobot/drones) and [jderobot-assets](https://github.com/JdeRobot/assets/tree/kinetic-devel).
- Unzip packages and move them to *catkin_ws/src*.
- Update ROS dependencies (see step 3.3).
- Clean (`catkin clean`) and re-build catkin (see step 3.4).
- Test an exercise. E.g. At academy/exercises/follow_road run `roslaunch follow_road.launch`.

***

# Troubleshooting
If you had troubles during installation or execution, you may be lucky and find a solution here.

## Problem with _ignite-tool_, _ignite-fuel_, _ignite-math_, _ignite-*_...
Make sure to be updated and upgraded.
```
sudo apt update
sudo apt upgrade
```

## Problem with catkin_ws dependencies
As it had been explained, make sure to have updated ROS dependencies.
```
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro melodic
rosdep install --from-paths . --ignore-src --rosdistro melodic -y
```

## Failure to install Geographiclib dataset
Links may be broken. Check this [issue](https://github.com/mavlink/mavros/issues/963).

## qfi package not found
Please follow qfi installation instructions [here](https://github.com/JdeRobot/ThirdParty/tree/master/qflightinstruments).
Make sure to run last instruction with superuser priviledges (´sudo make install´).

## During qfi installation, _unknown module in QT: svg_
Check [this](https://stackoverflow.com/questions/21098805/unknown-modules-in-qt-svg) out.

