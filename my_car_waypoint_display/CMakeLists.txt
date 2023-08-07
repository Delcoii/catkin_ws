cmake_minimum_required(VERSION 3.0.2)
project(my_car_waypoint_display)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp

  lanelet2_core
  lanelet2_projection
  lanelet2_io
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_odo
  CATKIN_DEPENDS nav_msgs roscpp
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#################################################################
add_executable(my_car_waypoint_publish
  src/odometry_pub_node.cpp
)

add_dependencies(my_car_waypoint_publish ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(my_car_waypoint_publish
  ${catkin_LIBRARIES}
)
#############################################################




add_executable(pub_lat_lon
  src/pub_lat_lon.cpp
)

add_dependencies(pub_lat_lon ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pub_lat_lon
  ${catkin_LIBRARIES}
)