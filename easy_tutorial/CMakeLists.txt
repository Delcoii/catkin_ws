cmake_minimum_required(VERSION 2.8.3)
project(easy_tutorial)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs
roscpp)
add_message_files(FILES MsgTutorial.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
LIBRARIES ros_tutorials_topic
CATKIN_DEPENDS std_msgs roscpp
)


include_directories(${catkin_INCLUDE_DIRS})

#pub
add_executable(topic_publisher src/topic_publisher.cpp)
add_dependencies(topic_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_publisher ${catkin_LIBRARIES})

#sub
add_executable(topic_subscriber src/topic_subscriber.cpp)
add_dependencies(topic_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_subscriber ${catkin_LIBRARIES})
