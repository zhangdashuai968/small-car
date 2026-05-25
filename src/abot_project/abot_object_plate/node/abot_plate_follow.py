#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy

import cv2 as cv
import numpy as np
import rospy
from hyperlpr import *
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from utils import CvFpsCalc
from utils import CvChDisp
from geometry_msgs.msg import Twist



class Plate_Dect:
    def __init__(self):
        image_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')

        self.getImageStatus = False

        self.color_image = Image()
        self.cvFpsCalc = CvFpsCalc(buffer_len=10)
        self.TextCh = CvChDisp()
        self.score = 0.85

        self.image_center_x = 320
        self.image_center_y = 240
        self.offset_x = 80
        self.follow_rect = 70*60
        self.offset_rect = 500
        self.linear_speed_max = 0.3
        self.angular_speed_max = 1


        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=52428800)
        self.image_pub = rospy.Publisher('/abot/detection_image',  Image, queue_size=1)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # if no image messages
        while (not self.getImageStatus) :
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):
        display_fps = self.cvFpsCalc.get()
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        self.color_image = cv.cvtColor(self.color_image, cv.COLOR_BGR2RGB)

        debug_image = copy.deepcopy(self.color_image)
        plate = HyperLPR_plate_recognition(self.color_image, 16, charSelectionDeskew=False)
        if len(plate) > 0:
            if plate[0][1] > self.score:
                PlateLT = (plate[0][2][0], plate[0][2][1])
                PlateRB = (plate[0][2][2], plate[0][2][3])
                w = plate[0][2][2] - plate[0][2][0]
                h = plate[0][2][3] - plate[0][2][1]
                cx = plate[0][2][0] + w/2
                cv.rectangle(debug_image, PlateLT, PlateRB, (0, 255, 0) , 1, 4)
                font_image = self.TextCh.text(debug_image, (300, 10), plate[0][0])
                debug_image = cv.cvtColor(font_image, cv.COLOR_BGR2RGB)
                if (cx < self.image_center_x - self.offset_x) or (cx > self.image_center_x + self.offset_x):
                    angular_z = self.GetAngular(self.image_center_x - cx)
                else:
                    angular_z = 0

                rect = w * h
                if(rect < self.follow_rect - self.offset_rect) or (rect > self.follow_rect + self.offset_rect):
                    linear_x = self.GetLinear((self.follow_rect - rect))
                else:
                    linear_x = 0
                rospy.loginfo("rect: %f, linear_x:%f, angular_z: %f", rect, linear_x, angular_z)
                twist = Twist()
                twist.linear.x = linear_x
                #twist.linear.x = 0
                twist.angular.z =angular_z
                self.vel_pub.publish(twist)

        self.publish_image(debug_image, image.height, image.width)


    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = "camera"
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)

    def GetLinear(self, offset):
        Kp = 0.0004
        Kd = 0.0001
        last_error = 0
        error = offset
        if abs(error) >= self.follow_rect:
            error = last_error

        speed = Kp*error + Kd*(error - last_error)
        last_error = error
        if speed > self.linear_speed_max:
            speed = self.linear_speed_max

        if speed < -self.linear_speed_max:
            speed = -self.linear_speed_max

        return speed

    def GetAngular(self, offset):
        Kp = 0.004
        Kd = 0.0002
        last_error = 0
        error = offset
        if abs(error) >= 320:
            error = last_error

        speed = Kp*error + Kd*(error - last_error)
        last_error = error
        if speed > self.angular_speed_max:
            speed = self.angular_speed_max

        if speed < -self.angular_speed_max:
            speed = -self.angular_speed_max

        return speed



def main():
    rospy.init_node('abot_plate', anonymous=True)
    plate_dect = Plate_Dect()
    rospy.spin()

if __name__ == '__main__':
    main()
