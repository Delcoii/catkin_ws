cmake_minimum_required(VERSION 3.0.2)
project(stanley_method_v3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    message_generation
    roscpp
    std_msgs
    tf
)


add_message_files(
    FILES
    CallbackMyCar.msg
    CarlaEgoVehicleControl.msg
    CarlaEgoVehicleStatus.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
    nav_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES stanley_method_v3
    CATKIN_DEPENDS geometry_msgs roscpp std_msgs
    DEPENDS system_lib
)

############################################
find_package(yaml-cpp REQUIRED)



###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)


add_executable(stanley_method_v3
    src/stanley.cpp
    src/pid.cpp
    src/sub_pub_class.cpp
    src/waypoint_save.cpp
    src/calculating_tools.cpp
)

add_dependencies(stanley_method_v3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(stanley_method_v3
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

