cmake_minimum_required(VERSION 2.8.3)
project(robocape_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/imu_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
