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


class Plate_Dect:
    def __init__(self):
        image_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')

        self.getImageStatus = False

        self.color_image = Image()
        self.cvFpsCalc = CvFpsCalc(buffer_len=10)
        self.TextCh = CvChDisp()
        self.score = 0.85

        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=52428800)
        self.image_pub = rospy.Publisher('/abot/detection_image',  Image, queue_size=1)


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
                cv.rectangle(debug_image, PlateLT, PlateRB, (0, 255, 0) , 1, 4)
                #cv2img = cv.cvtColor(debug_image, cv.COLOR_BGR2RGB)
                font_image = self.TextCh.text(debug_image, (300, 10), plate[0][0])
                debug_image = cv.cvtColor(font_image, cv.COLOR_BGR2RGB)
        cv.putText(debug_image, "FPS:" + str(display_fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv.LINE_AA)
        self.publish_image(debug_image, image.height, image.width)
        cv.imshow('Plate Detection Demo', debug_image)
        key = cv.waitKey(1)


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


def main():
    rospy.init_node('abot_plate', anonymous=True)
    plate_dect = Plate_Dect()
    rospy.spin()

if __name__ == '__main__':
    main()
