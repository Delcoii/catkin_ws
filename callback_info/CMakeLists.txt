cmake_minimum_required(VERSION 3.0.2)
project(callback_info)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  std_msgs
  tf
)

add_message_files(FILES
  CarlaEgoVehicleControl.msg
  CarlaEgoVehicleStatus.msg
  CallbackMyCar.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES callback_info
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
 DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/callback_info.cpp
# )
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(callback 
  src/callback.cpp
  src/car_status.cpp
)

add_dependencies(callback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(callback
  ${catkin_LIBRARIES}
)

