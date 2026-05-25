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
from abot_grasp_race.msg import ObjectInfo, ObjectInfoArray
from geometry_msgs.msg import Twist


"""
Object detection and point calculation module for ROS.

This module provides a class for detecting objects in camera images and calculating
their 3D coordinates using depth information.
"""


class ObjectVision():
    """
    A class for detecting objects in camera images and calculating their 3D coordinates.
    
    This class subscribes to camera and depth image topics, processes the images to
    detect objects of specific colors, and calculates their 3D coordinates using depth
    information.
    """
    def __init__(self):
        rospy.init_node('abot_object_xyz_node', log_level=rospy.INFO)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth_registered/image_raw')
        self.depth_info_topic = rospy.get_param('~depth_info_topic', '/camera/depth/camera_info')
        self.real_area = rospy.get_param('~real_area', '1000')
        self.out_y = rospy.get_param('~out_y', '390')

        self.offset = rospy.get_param('~depth_offset', '10')

        c1 = rospy.get_param("~color1", [16, 101, 171, 77, 255, 255, 'red'])
        c2 = rospy.get_param("~color2", [20, 100, 100, 30, 255, 255, 'yellow'])
        c3 = rospy.get_param("~color3", [35, 43, 35, 90, 255, 255, 'green'])
        self.K = rospy.get_param("~K", [570.3422241210938, 0.0, 314.5, 0.0, 570.3422241210938, 235.5, 0.0, 0.0, 1.0])

        self.color_dist = {
            c1[6]: {'Lower': np.array([c1[0], c1[1], c1[2]]), 'Upper': np.array([c1[3], c1[4], c1[5]])},
            c2[6]: {'Lower': np.array([c2[0], c2[1], c2[2]]), 'Upper': np.array([c2[3], c2[4], c2[5]])},
            c3[6]: {'Lower': np.array([c3[0], c3[1], c3[2]]), 'Upper': np.array([c3[3], c3[4], c3[5]])},
            }

        self.camera_model = os.getenv("CAMERA", "value does not exist")
        r = rospy.Rate(5)
        rospy.on_shutdown(self.shutdown)

        self.cvBridge = CvBridge()

        self.object_info = None
        self.depth_info = None
        self.depth_image = None
        self.cv_image = None
        self.cnt = 0

	im_sub = message_filters.Subscriber(self.camera_topic, Image)
        dep_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 10, 0.5)
        self.timeSynchronizer.registerCallback(self.ImageCallback)


        self.pub_object_image = rospy.Publisher('/object_image', Image, queue_size=1)
        
        self.pub_object_info = rospy.Publisher('/object_info', ObjectInfoArray, queue_size=1)

        while not rospy.is_shutdown():
            r.sleep()

    def FindObject(self, objs, info_id, img):
        c = max(objs, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        cv2.drawContours(img, [np.int0(box)], -1, (0, 255, 255), 2)
        cx, cy = rect[0]
        cx = int(cx)
        cy = int(cy)
        h, w = rect[1]
        angle = rect[2]
        if h*w > self.real_area and (cy + self.offset) < 480 and (cx + self.offset) < 640:
            obj = ObjectInfo()
            depth_arr = [self.depth_image[cy, cx], self.depth_image[cy+self.offset, cx+self.offset],  
                       self.depth_image[cy-self.offset, cx-self.offset], 
                       self.depth_image[cy+self.offset, cx],
                       self.depth_image[cy-self.offset, cx], self.depth_image[cy, cx+self.offset], 
                       self.depth_image[cy, cx-self.offset]]
            array_z = [x for x in depth_arr if math.isnan(x) == False]
            if self.camera_model == "intel" or  self.camera_model == "gemini":
                real_z = np.mean(array_z)
            else:
                real_z = np.mean(array_z)*1000
            real_x = (cx - self.K[2]) / self.K[0] * real_z 
            real_y = (cy - self.K[5]) / self.K[4] * real_z 
            if  math.isnan(real_z) is not True and real_z > 0:
                obj.obj_id = info_id
                obj.center_x = cx
                obj.center_y = cy
                obj.size_x = w
                obj.size_y =  h
                obj.angle =  angle
                obj.point_x = real_x
                obj.point_y = real_y
                obj.point_z = real_z
            
                return obj


    def ImageCallback(self, image_msg, depth_msg):
        self.cnt = self.cnt + 1
        if self.cnt == 3:
            self.cnt = 0
            img = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.cv_image = img[0:self.out_y]

            if self.camera_model == "intel":
                self.depth_image = self.cvBridge.imgmsg_to_cv2(depth_msg, '16UC1')
            else :
                self.depth_image = self.cvBridge.imgmsg_to_cv2(depth_msg, '32FC1')

            blurred = cv2.GaussianBlur(self.cv_image, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            erode_hsv = cv2.erode(hsv, None, iterations=2)
            objArray = ObjectInfoArray()
            objArray.detections =[]
            for i in self.color_dist:
                mask = cv2.inRange(erode_hsv, self.color_dist[i]['Lower'], self.color_dist[i]['Upper'])
                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                if len(cnts) > 0:
                    obj_msg = self.FindObject(cnts, i, self.cv_image) 
                    if obj_msg is not None:
                        cv2.circle(self.cv_image, (int(obj_msg.center_x), int(obj_msg.center_y)), 3, (216, 0, 255), -1)
                        objArray.detections.append(obj_msg)
                        self.pub_object_info.publish(objArray)

            self.pub_object_image.publish(self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))

    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown object_xyz_node ...")

if __name__ == '__main__':
    try:
        detect = ObjectVision()
        rospy.spin()
    except Exception, e:
        rospy.logerr("%s", str(e))
        os._exit(1)

