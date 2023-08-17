cmake_minimum_required(VERSION 3.0.2)
project(my_car_waypoint_display)


find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    std_msgs
    roscpp

    lanelet2_core
    lanelet2_projection
    lanelet2_io
)
find_package(Armadillo REQUIRED)



catkin_package(
    INCLUDE_DIRS include
    LIBRARIES waypoint_odo
    CATKIN_DEPENDS std_msgs nav_msgs roscpp
    DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
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
#############################################################
add_executable(target_velocity_raw
    src/pub_target_velocity.cpp

    include/my_car_waypoint_display/Filters/Filters.cpp
    include/my_car_waypoint_display/math_functions/math_functions.cpp
    include/my_car_waypoint_display/waypoint_save/waypoint_save_raw.cpp
)

add_dependencies(target_velocity_raw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(target_velocity_raw
    ${catkin_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
)
#############################################################

add_executable(target_velocity_cut
    src/pub_target_velocity.cpp

    include/my_car_waypoint_display/Filters/Filters.cpp
    include/my_car_waypoint_display/math_functions/math_functions.cpp
    include/my_car_waypoint_display/waypoint_save/waypoint_save_cut.cpp
)

add_dependencies(target_velocity_cut ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(target_velocity_cut
    ${catkin_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
)
