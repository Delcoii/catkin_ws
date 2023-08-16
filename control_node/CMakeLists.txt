cmake_minimum_required(VERSION 3.0.2)
project(control_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    tf

    message_generation

    visualization_msgs
    lanelet2_core
    lanelet2_projection
    lanelet2_io
)
find_package(Armadillo REQUIRED)


## Generate messages in the 'msg' folder
add_message_files( FILES
    CarlaEgoVehicleControl.msg
    CarlaEgoVehicleStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES
    std_msgs
    geometry_msgs
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES control_node
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf
    DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(control_node
    src/control.cpp

    include/control_node/waypoint_save/get_lat_lon.cpp
    include/control_node/ControlMsgs/ControlMsgs.cpp
    include/control_node/pid/pid.cpp
    include/control_node/stanley/stanley.cpp
    include/control_node/pure_pursuit/PurePursuitControl.cpp
    include/control_node/math_functions/math_functions.cpp
    include/control_node/error_calculate/error_calculate.cpp
    include/control_node/MovingAverage/MovingAverage.cpp
)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(control_node
    ${catkin_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
)
