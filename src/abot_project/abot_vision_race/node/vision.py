#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Authors: kevin

import rospy
import os
import math
import numpy as np
import cv2
import time
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Pose, Point, Quaternion
from vision_msgs.msg import BoundingBox2D, Detection2DArray,ObjectHypothesisWithPose,Detection2D
from playsound import playsound
from nav import *

class VisionRace():
    def __init__(self):

        self.task_image_pos = rospy.get_param("~task_image_pos")
        self.task_detect_pos = rospy.get_param("~task_detect_pos")
        self.task_pos = rospy.get_param("~task_pos")

        self.dir_path = os.path.dirname(os.path.realpath(__file__))

        self.dir_path = self.dir_path.replace('abot_vision_race/node', 'abot_vision_race/')
        self.dir_path += 'config/'

        r = rospy.Rate(5)
        rospy.on_shutdown(self.shutdown)

        self.task_image = []
        self.detect_image = None
        self.get_task_flag = 0
        self.task_detect_flag = 0
        self.task_finish_flag = 0
        self.cnt = 0
        self.detect_task_cnt = 0

        self.get_task_lock = False
        self.detect_task_start = False
        self.detect_task_lock = False
        self.detect_task_finish = False
        self.task_finish = False



        self.object_sub = rospy.Subscriber("/detect_node/detections", Detection2DArray, self.ObjectCallback)

        self.cvBridge = CvBridge()
        self.nav = Nav()

        while not rospy.is_shutdown():
            if self.get_task_lock is False:
                self.VisionNavGetTask() 
            if self.detect_task_start is True and self.detect_task_lock is False and self.detect_task_finish is False:
                self.VisionNavDetectTask()
            if self.detect_task_finish is True and self.task_finish is False:
                self.VisionNavTaskPos()

            r.sleep()

    def VisionNavGetTask(self):
        rospy.loginfo("start get task image ")
        self.nav.SendGoal(self.NavPoseConvert(self.task_image_pos))
        state =self.nav.NavState()
        rospy.loginfo("nav status is %d", state)
        if state == 1 :
            self.get_task_lock = True

    def VisionNavDetectTask(self):
        rospy.loginfo("start task image detect")
        self.nav.SendGoal(self.NavPoseConvert(self.task_detect_pos[self.detect_task_cnt]))
        state =self.nav.NavState()
        rospy.loginfo("nav status is %d", state)
        if state == 1 :
            self.detect_task_lock = True
            rospy.loginfo("detect task start %d", self.detect_task_cnt)

    def VisionNavTaskPos(self):
        rospy.loginfo("start nav Task Pos")
        self.nav.SendGoal(self.NavPoseConvert(self.task_pos[self.detect_task_cnt]))
        state =self.nav.NavState()
        if state == 1 :
            rospy.loginfo("task finish ")
            self.task_finish = True
            self.voice_play("task_ok")

    def NavPoseConvert(self, pos):
        q = quaternion_from_euler(0, 0, np.deg2rad(pos[2]))

        return Pose(Point(pos[0], pos[1], 0), Quaternion(q[0], q[1], q[2], q[3]))

    def voice_play(self, name):
        mp3_file = self.dir_path + name + ".mp3"
        if os.access(mp3_file, os.F_OK):
            playsound(mp3_file)

    def ObjectCallback(self, msg):
        if self.get_task_lock is True and self.get_task_flag == 0:
            if self.cnt == 20 :
                for obj in msg.detections:
                    self.task_image.append(obj.header.frame_id)
                print(self.task_image)
                self.get_task_flag = 1
                self.cnt = 0
                self.detect_task_start = True
            self.cnt = self.cnt + 1
        if self.detect_task_lock is True and self.task_finish_flag == 0:
            if self.cnt == 10 :
                print(msg.detections[-1].header.frame_id)
                self.detect_image = msg.detections[-1].header.frame_id
                self.voice_play(self.detect_image)
                #playsound(self.dir_path + self.detect_image + ".mp3")
                if self.detect_image in self.task_image:
                    self.detect_task_finish = True
                    self.task_finish_flag = 1
                    self.voice_play("detect_ok")
                    #playsound(self.dir_path + "detect_ok.mp3")
                    rospy.loginfo("detect task success")
                else:
                    self.detect_task_cnt = self.detect_task_cnt + 1
                    self.detect_task_lock = False
                    rospy.loginfo("next task start %d", self.detect_task_cnt)
                self.cnt = 0
            self.cnt = self.cnt + 1


    def main(self):
        rospy.spin()

    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown node ...")


if __name__ == '__main__':
    rospy.init_node('vision_race_node')
    node = VisionRace()
    node.main()
