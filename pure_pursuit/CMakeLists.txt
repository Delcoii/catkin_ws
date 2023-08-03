cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
)


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
    LIBRARIES pure_pursuit
    CATKIN_DEPENDS roscpp
    DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_executable(pure_pursuit
    src/main.cpp
    src/PurePursuitControl.cpp
    
    include/pure_pursuit/ControlMsgs/ControlMsgs.cpp
    include/pure_pursuit/pid/pid.cpp
    include/pure_pursuit/waypoint_save/waypoint_save.cpp
    
)

add_dependencies(pure_pursuit ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)


target_link_libraries(pure_pursuit
    ${catkin_LIBRARIES}
)
