cmake_minimum_required(VERSION 2.8.3)
project(abot_yolo_detection)

add_compile_options(--std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    image_transport
    sensor_msgs
)

find_package(OpenCV 4 REQUIRED)

set(DARKNET_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/darknet/include)
set(DARKNET_LIBRARY ${PROJECT_SOURCE_DIR}/darknet/libdarknet.so)

catkin_package(
#    CATKIN_DEPENDS roscpp cv_bridge image_transport  sensor_msgs
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${DARKNET_INCLUDE_DIR}
)
    
add_executable(abot_yolo_detection_node
     src/main.cpp 
     src/abot_yolo_detection_node.cpp
     )
target_link_libraries(abot_yolo_detection_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${DARKNET_LIBRARY}
    )
