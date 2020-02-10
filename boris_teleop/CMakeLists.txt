cmake_minimum_required(VERSION 2.8.3)
project(boris_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS rospy geometry_msgs
  DEPENDS
)

catkin_python_setup()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/boris_teleop_key
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
