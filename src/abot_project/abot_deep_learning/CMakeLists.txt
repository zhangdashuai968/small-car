
cmake_minimum_required(VERSION 2.8.3)
project(abot_deep_learning)

# locate dependencies
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  cv_bridge
  vision_msgs
  std_msgs
)

find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(Qt4 REQUIRED)

#include(${QT_USE_FILE})
#add_definitions(${QT_DEFINITIONS})


# define catkin package
catkin_package(
    CATKIN_DEPENDS roscpp image_transport sensor_msgs  cv_bridge
)

# include paths
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

link_directories(/usr/lib/aarch64-linux-gnu/tegra)

# enable c++11 (TensorRT requirement)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# inference nodes
add_executable(abot_classfiy src/node_imagenet.cpp src/image_converter.cpp)
target_link_libraries(abot_classfiy ${catkin_LIBRARIES} jetson-inference)

add_executable(abot_detect src/node_detectnet.cpp src/image_converter.cpp)
target_link_libraries(abot_detect ${catkin_LIBRARIES} jetson-inference  ${OpenCV_LIBS})

#add_executable(rikibot_seg src/node_segnet.cpp src/image_converter.cpp)
#target_link_libraries(rikibot_seg ${catkin_LIBRARIES} jetson-inference)


#if(catkin_EXPORTED_LIBRARIES)
#    add_dependencies(ros_deep_learning_nodelets ${catkin_EXPORTED_LIBRARIES})
#endif()

