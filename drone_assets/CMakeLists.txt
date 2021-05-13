cmake_minimum_required(VERSION 3.0.2)
project(drone_assets)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

include_directories()

install(DIRECTORY
  models
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
