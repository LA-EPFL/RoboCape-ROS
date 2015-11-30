cmake_minimum_required(VERSION 2.8.3)
project(robocape_ublox)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/configure.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
