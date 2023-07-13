cmake_minimum_required(VERSION 3.0.2)
project(waypoint_odo)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_odo
  CATKIN_DEPENDS nav_msgs roscpp
  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(odo_pub_node src/waypoint_odo_node.cpp)

add_dependencies(odo_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(odo_pub_node
  ${catkin_LIBRARIES}
)
