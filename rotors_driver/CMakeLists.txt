cmake_minimum_required(VERSION 3.0.2)
project(rotors_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  mavros_msgs
  mavros
  nav_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  tf
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/rotors_driver_node.py
  scripts/rotors_land
  scripts/rotors_takeoff
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)