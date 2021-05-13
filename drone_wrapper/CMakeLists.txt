cmake_minimum_required(VERSION 3.0.2)
project(drone_wrapper)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  mavros_msgs
  rospy
  sensor_msgs
  tf
  mavros
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
	scripts/play_python_code
	scripts/load_parameters
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
