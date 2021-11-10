cmake_minimum_required(VERSION 3.0.2)
project(cl_drone_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  drone_wrapper
)

catkin_package()

install(PROGRAMS
  src/cl_drone_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
