cmake_minimum_required(VERSION 2.8.3)
project(robocape_bringup)

find_package(catkin REQUIRED)

catkin_package(
 #  INCLUDE_DIRS include
 #  LIBRARIES robocape_bringup
 #  CATKIN_DEPENDS other_catkin_pkg
 #  DEPENDS system_lib
)

find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)
