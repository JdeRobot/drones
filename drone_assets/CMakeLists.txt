cmake_minimum_required(VERSION 2.8.3)
project(drone_assets)

## ROS INSTALL ##

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY models urdf
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## JDEROBOT INSTALL ##

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (CMAKE_INSTALL_PREFIX /opt/jderobot/share/jderobot/
         CACHE PATH "default install path" FORCE)
endif()

set (MODELS ${CMAKE_INSTALL_PREFIX}/gazebo/models)

configure_file (
    assets-setup.sh.in
    assets-setup.sh
)

install(DIRECTORY models
DESTINATION gazebo)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/assets-setup.sh 
DESTINATION gazebo)


include(cpackit.cmake)
