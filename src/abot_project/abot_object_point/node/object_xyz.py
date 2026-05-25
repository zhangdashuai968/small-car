#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import numpy as np
import os,cv2
import math
import imutils
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from vision_msgs.msg import BoundingBox2D, Detection2DArray,ObjectHypothesisWithPose,Detection2D
from geometry_msgs.msg import Twist


class RikibotObjectDepth():
    def __init__(self):
        rospy.init_node('abot_object_xyz_node', log_level=rospy.INFO)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth_registered/image_raw')
        self.depth_info_topic = rospy.get_param('~depth_info_topic', '/camera/depth/camera_info')
        self.real_area = rospy.get_param('~real_area', '1000')
        self.depth_offset = rospy.get_param('~depth_offset', '10')
        self.camera_model = os.getenv("CAMERA", "value does not exist")
        r = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)

        self.pub_object_type = "raw"
	self.sub_image_type = "raw"
        self.cvBridge = CvBridge()
        self.object_info = None
        self.depth_info = None
        self.depth_image = None
        self.cv_image = None

	im_sub = message_filters.Subscriber(self.camera_topic, Image)
        dep_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 10, 0.5)
        self.timeSynchronizer.registerCallback(self.ImageCallback)


        self.object_sub = rospy.Subscriber("/detect_node/detections", Detection2DArray, self.ObjectCallback)
        self.depth_info_sub = rospy.Subscriber(self.depth_info_topic, CameraInfo, self.DepthInfoCallback)

        if self.pub_object_type == "compressed":
            self.pub_object_image = rospy.Publisher('/object_image/compressed', CompressedImage, queue_size=1)
        else:
            self.pub_object_image = rospy.Publisher('/object_point_image', Image, queue_size=1)
        
        self.pub_object_msg = rospy.Publisher('/object_point_info', Detection2D, queue_size=1)

        while not rospy.is_shutdown():
            r.sleep()

    def ImageCallback(self, image_msg, depth_msg):
        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        if self.camera_model == "intel":
            self.depth_image = self.cvBridge.imgmsg_to_cv2(depth_msg, '16UC1')
        else :
            self.depth_image = self.cvBridge.imgmsg_to_cv2(depth_msg, '32FC1')


    def ObjectCallback(self, msg):
        self.object_info = msg
        object_msg = Detection2D()

        object_msg.results = []
        if self.depth_image is not None and self.cv_image is not None:
            for obj in self.object_info.detections:
                hyp = ObjectHypothesisWithPose()
                img_x = int(obj.bbox.center.x)
                img_y =  int(obj.bbox.center.y)
                size_x = int(obj.bbox.size_x)
                size_y = int(obj.bbox.size_y)
                if size_x*size_y > self.real_area :
                    depth_arr = [self.depth_image[img_y, img_x], self.depth_image[img_y+self.depth_offset, img_x+self.depth_offset],  
                               self.depth_image[img_y-self.depth_offset, img_x-self.depth_offset], 
                               self.depth_image[img_y+self.depth_offset, img_x],
                               self.depth_image[img_y-self.depth_offset, img_x], self.depth_image[img_y, img_x+self.depth_offset], 
                               self.depth_image[img_y, img_x-self.depth_offset]]
                    array_z = [x for x in depth_arr if math.isnan(x) == False]
                    if self.camera_model == "intel" or self.camera_model == "gemini" :
                        real_z = np.mean(array_z)
                    else:
                        real_z = np.mean(array_z)*1000
                    real_x = (img_x - self.depth_info.K[2]) / self.depth_info.K[0] * real_z 
                    real_y = (img_y - self.depth_info.K[5]) / self.depth_info.K[4] * real_z 
                    if math.isnan(real_z) is False:
                        text = "x:%d,"%real_x + " y:%d,"%real_y + "z:%.d"%real_z
                        object_msg.header.frame_id = obj.header.frame_id 
                        hyp.id = obj.results[-1].id
                        hyp.score = obj.results[-1].score
                        hyp.pose.pose.position.x = real_x
                        hyp.pose.pose.position.y = real_y
                        hyp.pose.pose.position.z = real_z
                        object_msg.results.append(hyp)
                        rospy.loginfo("image pos x: %d, y: %d, real object xyz: %d, %d, %d", img_x, img_y, real_x, real_y, real_z)
                        cv2.putText(self.cv_image, text, (img_x-50, img_y+50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                        cv2.rectangle(self.cv_image, (img_x-size_x/2, img_y-size_y/2), (img_x+size_x/2, img_y+size_y/2), (0, 255, 0), 2)
                        cv2.circle(self.cv_image, (int(img_x), int(img_y)), 3, (216, 0, 255), -1)
                        self.pub_object_msg.publish(object_msg)
                    else:
                        rospy.loginfo("no object detect!!!!!!!!!!!!!!!!!!!!")
            self.pub_object_image.publish(self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))

    def DepthInfoCallback(self, depthinfo_msg):
        self.depth_info = depthinfo_msg
        #rospy.loginfo("size: %f, center: %f", self.object_info.detections[-1].bbox.size_x, self.object_info.detections[-1].bbox.center.x)  

    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown object_xyz_node ...")

if __name__ == '__main__':
    try:
        detect = RikibotObjectDepth()
        rospy.spin()
    except Exception, e:
        rospy.logerr("%s", str(e))
        os._exit(1)

