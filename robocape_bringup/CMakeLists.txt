cmake_minimum_required(VERSION 2.8.3)
project(robocape_bringup)

find_package(catkin REQUIRED)

##   * add a build_depend tag for "message_generation"
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:

## Declare ROS dynamic reconfigure parameters ##

## To declare and build dynamic reconfigure parameters within this
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
##   * add "dynamic_reconfigure" to
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg

catkin_package(
#  INCLUDE_DIRS include
 #  LIBRARIES robocape_bringup
 #  CATKIN_DEPENDS other_catkin_pkg
 #  DEPENDS system_lib

)

## Declare a C++ library

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(robocape_bringup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(robocape_bringup_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

