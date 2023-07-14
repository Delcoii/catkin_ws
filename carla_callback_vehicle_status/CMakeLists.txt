cmake_minimum_required(VERSION 3.0.2)
project(carla_callback_vehicle_status)

add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs

    message_generation
)

add_message_files(FILES
    CarlaEgoVehicleControl.msg
    CarlaEgoVehicleStatus.msg
)

generate_messages (DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES carla_callback_vehicle_status
    CATKIN_DEPENDS roscpp std_msgs
    #DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(status_callback_node src/callback_by_rosinfo.cpp)

add_dependencies(status_callback_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(status_callback_node
    ${catkin_LIBRARIES}
)
