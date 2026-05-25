#ifndef RIKIBOT_YOLO_DETECTION_NODE_HPP
#define RIKIBOT_YOLO_DETECTION_NODE_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "yolo_v2_class.hpp"



namespace abot_yolo_detection
{

class AbotYoloDetectionNode
{

public:
    AbotYoloDetectionNode();
    ~AbotYoloDetectionNode();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg_rgb);
    void run();
    void drawBoxes(cv::Mat mat_img);

private:
    ros::NodeHandle nh, privateNh;
    image_transport::ImageTransport it;
    image_transport::Subscriber rgbSub;
    image_transport::Publisher  image_pub;
    ros::Publisher detection_pub;
    Detector *yoloDetector; // use smart ptr instead
    std::vector<std::string> objectsNames;
    std::vector<bbox_t> objects;
};

} // namespace abot_yolo_detection

#endif // RIKIBOT_YOLO_DETECTION_NODE_HPP
