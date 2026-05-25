#include <ros/ros.h>
#include <abot_yolo_detection_node.hpp>

using namespace abot_yolo_detection;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "abot_yolo_detection_node");

  AbotYoloDetectionNode node;

  node.run();
  
  return 0;
}
