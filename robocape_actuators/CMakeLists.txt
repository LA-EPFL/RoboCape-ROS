cmake_minimum_required(VERSION 2.8.3)
project(robocape_actuators)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
)

catkin_package(
	# INCLUDE_DIRS include
	# LIBRARIES actuators
	# CATKIN_DEPENDS roscpp rospy std_msgs
	# DEPENDS system_lib
)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

# add_executable(actuator src/actuator.cpp)
# target_link_libraries(actuator ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/actuator_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
