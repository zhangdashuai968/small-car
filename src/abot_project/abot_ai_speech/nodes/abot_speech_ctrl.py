#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: kevin <304050118@qq.com>

import rospy
import os
import sys
import time
from threading import Timer
from audio_common_msgs.msg import AudioData
from abot_ai_speech import AbotAISpeech
from geometry_msgs.msg import Twist

class AbotVoice(object):
    def __init__(self):
        self.sub_audio = rospy.Subscriber("speech_audio", AudioData, self.audio_cb)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.dir_path = self.dir_path.replace('abot_ai_speech/nodes', 'abot_ai_speech/')
        self.dir_path += 'voice/'
        self.device_id = rospy.get_param("~device_id", 0)
        self.timeout = rospy.get_param("~stop_time", 5)
        self.order = [u'前进',u'后退',u'左转',u'右转', u'停止']

        self.ai = AbotAISpeech(self.device_id, self.dir_path)
	self.audio_text = None
        self.is_canceling = False
        self.linear_x = 0.0
        self.angular_z = 0.0

        r = rospy.Rate(10)
        move_cmd = Twist()
        while not rospy.is_shutdown():
            move_cmd.linear.x = self.linear_x
            move_cmd.angular.z = self.angular_z
            self.pub_vel.publish(move_cmd)
            r.sleep()


    def cbStopvel(self):
        rospy.loginfo("Abot Stop")
        self.linear_x = 0.0
        self.angular_z = 0.0

    def audio_cb(self, msg):
        if self.is_canceling is True:
            return

        self.audio_text = self.ai.audio_to_text(msg.data)	
        if self.audio_text is not None:
            rospy.loginfo("You Said: %s", self.audio_text.encode('utf-8'))
            self.is_canceling = True
            if self.audio_text.find(self.order[0]) != -1:
                rospy.loginfo("Abot Go Forward")
                self.linear_x = 0.3
                self.angular_z = 0.0
                Timer(self.timeout, self.cbStopvel).start()
            if self.audio_text.find(self.order[1]) != -1:
                rospy.loginfo("Abot Go Back")
                self.linear_x = -0.3
                self.angular_z = 0.0
                Timer(self.timeout, self.cbStopvel).start()
            if self.audio_text.find(self.order[2]) != -1:
                rospy.loginfo("Abot Go Left")
                self.linear_x = 0.0
                self.angular_z = 1.0
                Timer(self.timeout, self.cbStopvel).start()
            if self.audio_text.find(self.order[3]) != -1:
                rospy.loginfo("Abot Go Right")
                self.linear_x = 0.0
                self.angular_z = -1.0
                Timer(self.timeout, self.cbStopvel).start()
            if self.audio_text.find(self.order[4]) != -1:
                rospy.loginfo("Abot Stop")
                self.linear_x = 0.0
                self.angular_z = 0.0
            self.is_canceling = False



if __name__ == '__main__':
    rospy.init_node("Abot_Voice_Node")
    rikibotai = AbotVoice()
    rospy.spin()
