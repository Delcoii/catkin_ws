cmake_minimum_required(VERSION 3.0.2)
project(cv_practice)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    #message_generation

    cv_bridge
    image_transport 
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES cv_practice
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge
#  DEPENDS system_lib
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)



add_executable(${PROJECT_NAME}cv_subscriber src/cv_sub_practice.cpp)

add_dependencies(${PROJECT_NAME}cv_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}cv_subscriber
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)