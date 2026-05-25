#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: kevin <304050118@qq.com>

import rospy
import os
import sys
import time
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
from rikibot_ai_speech import RikibotAISpeech

class RikibotVoice(object):
    def __init__(self):

        self.sub_audio = rospy.Subscriber("speech_audio", AudioData, self.audio_cb)
         
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.dir_path = self.dir_path.replace('abot_ai_speech/nodes', 'abot_ai_speech/')
        self.dir_path += 'voice/'
        self.device_id = rospy.get_param("~device_id", 0)

        self.ai = RikibotAISpeech(self.device_id, self.dir_path)
	self.audio_text = None
        self.is_voice = False
        self.is_canceling = False
        self.greet_word = "你好，很高兴认识你，我是小锐同学"
        self.greet_mode = False
        
        self.name = self.last_name = None
        self.sub_face_name = rospy.Subscriber("/face_name", String, self.name_cb)


        r = rospy.Rate(10)
        rospy.loginfo("Ready Listen........")
        self.begin = self.end = rospy.get_time()
        while not rospy.is_shutdown():
            if self.is_voice is True and self.greet_mode is False:
                self.is_voice = False
                self.is_canceling = True
                rospy.loginfo("You Said: %s", self.audio_text.encode('utf-8'))
                response_text = self.ai.speak_text(self.audio_text)
                rospy.loginfo("AI Said: %s", response_text.encode('utf-8'))
                self.ai.text_to_audio(response_text)
                self.ai.audio_play()
                time.sleep(1)
                self.is_canceling = False
                rospy.loginfo("Ready Listen........")
            if self.greet_mode is True:
                self.ai.text_to_audio(self.name + self.greet_word)
                self.ai.audio_play()
                self.last_name = self.name
                self.greet_mode = False
                self.end = rospy.get_time()


            r.sleep()

    def audio_cb(self, msg):
        if self.is_canceling is True:
            return

        self.audio_text = self.ai.audio_to_text(msg.data)	
        if self.audio_text is not None:
            self.is_voice = True

    def name_cb(self, name_msg):
        self.name = name_msg.data
        self.begin = rospy.get_time()
        if self.last_name != self.name:
            self.greet_mode = True

        if self.begin - self.end > 10:
            self.greet_mode = True

        #self.ai.text_to_audio(self.name + self.greet_word)
        #self.ai.audio_play()




if __name__ == '__main__':
    rospy.init_node("Rikibot_Voice_Node")
    rikibotai = RikibotVoice()
    rospy.spin()
