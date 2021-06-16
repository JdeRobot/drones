cmake_minimum_required(VERSION 3.0.2)
project(rqt_ground_robot_teleop)

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
	scripts/rqt_ground_robot_teleop
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY resource
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
	plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
