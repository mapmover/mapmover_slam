cmake_minimum_required(VERSION 3.0.2)
project(mapmover_mapping)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  nav_msgs
  visualization_msgs
  tf
  message_filters
  laser_geometry
  message_generation
  std_srvs)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen3 REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MapmoverDebugInfo.msg
  MapmoverIterData.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ResetMapping.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mapmover_mapping
  CATKIN_DEPENDS roscpp nav_msgs visualization_msgs tf message_filters laser_geometry message_runtime
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(include/mapmover_slam_lib)
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(mapmover_mapping
  src/MapmoverDebugInfoProvider.h
  src/MapmoverDrawings.h
  src/MapmoverMappingRos.h
  src/MapmoverMappingRos.cpp
  src/main.cpp
  src/PoseInfoContainer.cpp
  src/PoseInfoContainer.h
)

add_dependencies(mapmover_mapping mapmover_mapping_generate_messages_cpp)

target_link_libraries(mapmover_mapping
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS mapmover_mapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Don't ask why it's mapmover_slam_lib, this was Stefan Kohlbrecher's first ROS package and a wrapper of a pre ROS header only library
install(DIRECTORY include/mapmover_slam_lib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)
