cmake_minimum_required(VERSION 2.8.3)
project(drone_wrapper)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  mavros_msgs
  rospy
  sensor_msgs
  tf
  mavros
  gazebo_ros
)

catkin_python_setup()

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
