cmake_minimum_required(VERSION 3.0.2)
project(mapmover_map_server)

find_package(catkin REQUIRED COMPONENTS roscpp mapmover_map_tools mapmover_marker_drawing mapmover_nav_msgs nav_msgs tf)

catkin_package(
  CATKIN_DEPENDS roscpp mapmover_map_tools mapmover_marker_drawing mapmover_nav_msgs nav_msgs tf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mapmover_map_server src/mapmover_map_server.cpp)
add_dependencies(mapmover_map_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapmover_map_server
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS mapmover_map_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
