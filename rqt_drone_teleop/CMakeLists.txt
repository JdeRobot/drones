cmake_minimum_required(VERSION 2.8.3)
project(rqt_drone_teleop)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	rospy
	rqt_gui
	rqt_gui_py
	sensor_msgs
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
	scripts/rqt_drone_teleop
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY resource perspectives launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
	plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
