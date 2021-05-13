cmake_minimum_required(VERSION 3.0.2)
project(rqt_drone_teleop)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	rospy
	rqt_gui
	rqt_gui_py
	sensor_msgs
	drone_wrapper
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
	scripts/rqt_vel_teleop
	scripts/rqt_cam_viewer
	scripts/rqt_pos_teleop
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY resource perspectives
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
	plugin_vel_teleop.xml
	plugin_cam_viewer.xml
	plugin_pos_teleop.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
