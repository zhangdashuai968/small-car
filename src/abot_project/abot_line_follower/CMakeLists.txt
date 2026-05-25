cmake_minimum_required(VERSION 2.8.3)
project(abot_line_follower)

add_definitions(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy std_msgs genmsg geometry_msgs)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
#add_message_files()

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

## Executables
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(abot_line_node src/rikibot_line.cpp src/rikibot_line_detect.cpp)
target_link_libraries(abot_line_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(rikibot_line_node)

