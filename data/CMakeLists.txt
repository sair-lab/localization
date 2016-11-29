cmake_minimum_required(VERSION 2.8.3)
project(data)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg tf)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Num.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

# %EndTag(FULLTEXT)%

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(path src/talk.cpp)
target_link_libraries(path ${catkin_LIBRARIES})
add_dependencies(path check_generate_messages_cpp)


