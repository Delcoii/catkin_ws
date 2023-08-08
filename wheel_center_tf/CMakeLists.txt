cmake_minimum_required(VERSION 3.0.2)
project(wheel_center_tf)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    geometry_msgs
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES wheel_center_tf
    CATKIN_DEPENDS roscpp tf
    DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_executable(wheel_center_tf
    src/publish_wheel_center_pose.cpp
)

add_dependencies(wheel_center_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(wheel_center_tf
  ${catkin_LIBRARIES}
)
