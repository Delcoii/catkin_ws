cmake_minimum_required(VERSION 3.0.2)
project(marker_array_tutorial)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES marker_tutorial
  CATKIN_DEPENDS roscpp visualization_msgs
  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)


add_executable(mark_arr src/marker_array_tutorial_node.cpp)
add_dependencies(mark_arr ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(mark_arr ${catkin_LIBRARIES})
