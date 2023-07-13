cmake_minimum_required(VERSION 3.0.2)
project(marker_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_shapes src/basic_shapes.cpp)
add_dependencies(basic_shapes ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(basic_shapes ${catkin_LIBRARIES})