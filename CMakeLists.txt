cmake_minimum_required(VERSION 3.0.2)
project(ur5_tasks)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  moveit_kinematics
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur5_tasks
  CATKIN_DEPENDS roscpp rospy std_msgs trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  include/Reflexxes
  ${catkin_INCLUDE_DIRS}
)
add_library(ReflexxesLib SHARED IMPORTED)
set_property(TARGET ReflexxesLib PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libReflexxesTypeII.a)


add_executable(task2 src/task2.cpp)
add_dependencies(task2 ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(task2
  ${catkin_LIBRARIES} ReflexxesLib
)
#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/task1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})