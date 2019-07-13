#!/bin/bash

# This script assumes that ROS and Gazebo have already been installed

echo 'Installing MAVROS and GeographicLib'
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

echo 'Removing modemmanager'
sudo apt-get remove modemmanager

echo 'Installing Common Dependancies'
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool -y

# Install xxd (package depends on version)
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y

# Required python packages
sudo apt-get install python-argparse \
    python-empy python-toml python-numpy python-yaml \
    python-dev python-pip -y
sudo -H pip install --upgrade pip 
sudo -H pip install pandas jinja2 pyserial cerberus
sudo -H pip install pyulog

echo 'Installing Ninja'
sudo apt-get install ninja-build -y

echo 'Geting FastRTPS and FastCDR'
wget https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-7-1/eprosima_fastrtps-1-7-1-linux-tar-gz -O eprosima_fastrtps-1-7-1-linux.tar.gz
tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz eProsima_FastRTPS-1.7.1-Linux/
tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz requiredcomponents
tar -xzf requiredcomponents/eProsima_FastCDR-1.0.8-Linux.tar.gz

echo 'Building FastRTPS and FastCDR'
(cd eProsima_FastCDR-1.0.8-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
(cd eProsima_FastRTPS-1.7.1-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
rm -rf requiredcomponents eprosima_fastrtps-1-7-1-linux.tar.gz

echo 'Setting up catkin workspace'
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/PX4/Firmware.git
cd Firmware
git submodule update --init --recursive
cd ..
ln -s Firmware/Tools/sitl_gazebo mavlink_sitl_gazebo
cd ..
catkin build
echo 'source '$PWD'/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc

echo 'Installation of MAVROS and PX4 Complete. Test using: roslaunch px4 mavros_posix_sitl.launch'
