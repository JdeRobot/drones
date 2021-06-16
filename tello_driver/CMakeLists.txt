cmake_minimum_required(VERSION 2.8.3)
project(tello_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  geometry_msgs
  mavros_msgs
  mavros
  sensor_msgs
)

catkin_python_setup()
catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/tello_driver_node.py
  scripts/tello_land
  scripts/tello_quit
  scripts/tello_takeoff
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)