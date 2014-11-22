cmake_minimum_required(VERSION 2.8.3)
project(mico_rr_bridge)

find_package(catkin 
	REQUIRED COMPONENTS
	rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
	rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/MicoRRService.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(
#  DIRECTORY scripts/
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  USE_SOURCE_PERMISSIONS
#)
#
#install(
#  DIRECTORY share/
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  USE_SOURCE_PERMISSIONS
#)
#
#install(
#  DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  USE_SOURCE_PERMISSIONS
#)
