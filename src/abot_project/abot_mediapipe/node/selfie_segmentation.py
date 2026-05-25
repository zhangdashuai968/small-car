#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy

import cv2 as cv
import numpy as np
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from utils import CvFpsCalc


class Segment:
    def __init__(self):
        image_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')

        self.getImageStatus = False

        self.model_selection = 0
        self.score_th = 0.1
        self.bg_image = None


        self.mp_selfie_segmentation = mp.solutions.selfie_segmentation
        self.selfie_segmentation = self.mp_selfie_segmentation.SelfieSegmentation(model_selection=self.model_selection)

        self.color_image = Image()
        self.cvFpsCalc = CvFpsCalc(buffer_len=10)

        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=52428800)
        self.image_pub = rospy.Publisher('/rikibot/detection_image',  Image, queue_size=1)

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
        results = self.selfie_segmentation.process(self.color_image)

        # 描画 ################################################################
        mask = np.stack((results.segmentation_mask, ) * 3, axis=-1) >= self.score_th

        if self.bg_image is None:
            bg_resize_image = np.zeros(self.color_image.shape, dtype=np.uint8)
            bg_resize_image[:] = (0, 255, 0)
        else:
            bg_resize_image = cv.resize(bg_image,
                                        (self.color_image.shape[1], self.color_image.shape[0]))
        debug_image = np.where(mask, debug_image, bg_resize_image)

        cv.putText(debug_image, "FPS:" + str(display_fps), (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2,
                   cv.LINE_AA)
        self.publish_image(debug_image, image.height, image.width)

        cv.imshow('MediaPipe Selfie Segmentation Demo', debug_image)
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
    rospy.init_node('rikibot_mediapipe', anonymous=True)
    seg = Segment()
    rospy.spin()

if __name__ == '__main__':
    main()
