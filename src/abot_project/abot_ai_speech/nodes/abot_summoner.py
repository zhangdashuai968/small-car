#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: kevin <304050118@qq.com>

import rospy
import os
import sys
import time
import tf
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from math import radians, copysign
from transform_utils import quat_to_angle, normalize_angle
from abot_ai_speech import AbotAISpeech

class AbotTurnAngle(object):

    def __init__(self):
         self.obj_angle = radians(180)
         self.speed = 0.5
         self.tolerance = radians(1)
         self.base_frame = '/base_link'
         self.odom_frame = '/odom'
         self.odom_angular_scale_correction = 1.0
         self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
         self.r = rospy.Rate(20)


    def action(self, angle):
        rospy.loginfo("Start Turn Around")
	self.obj_angle = radians(angle)
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)
	self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))

	self.odom_angle = self.get_odom_angle()
	last_angle = self.odom_angle
	turn_angle = 0 
	error = self.obj_angle - turn_angle
	while abs(error) > self.tolerance :
	    # Rotate the robot to reduce the error
	    move_cmd = Twist()
	    move_cmd.angular.z = copysign(self.speed, error)
	    self.cmd_vel.publish(move_cmd)
	    self.r.sleep()

	    # Get the current rotation angle from tf                   
	    self.odom_angle = self.get_odom_angle()

	    # Compute how far we have gone since the last measurement
	    #delta_angle = self.odom_angular_scale_correction * normalize_angle(self.odom_angle - last_angle)
	    delta_angle = normalize_angle(self.odom_angle - last_angle)

	    # Add to our total angle so far
	    turn_angle += delta_angle

	    # Compute the new error
	    error = self.obj_angle - turn_angle

	    # Store the current angle for the next comparison
	    last_angle = self.odom_angle

	# Stop the robot
	self.cmd_vel.publish(Twist())

    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
    
        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))


class AbotSummoner(object):
    def __init__(self):
        self.sub_audio = rospy.Subscriber("speech_audio", AudioData, self.audio_cb)
        self.sub_direction = rospy.Subscriber("sound_direction", Int32, self.direction_cb)
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.dir_path = self.dir_path.replace('abot_ai_speech/nodes', 'abot_ai_speech/')
        self.device_id = rospy.get_param("~device_id", 0)
        self.dir_path += 'voice/'

        self.ai = AbotAISpeech(self.device_id, self.dir_path)
	self.audio_text = None
        self.doa = 0
        self.is_voice = False
        self.is_canceling = False
        self.offset_angle = 90
	self.turn = AbotTurnAngle()

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_voice is True:
                self.ai.text_to_audio(u"小图在的")
                self.ai.audio_play()
                if self.doa <= 0 and self.doa > -90:
                    angle = -self.offset_angle  - self.doa
                if self.doa <=-90 and self.doa >-180:
                    angle = -self.doa - self.offset_angle 
                if self.doa >0 and self.doa <=90:
                    angle = -self.offset_angle  - self.doa
                if self.doa >90 and self.doa <=180:
                    angle = 270 - self.doa
                rospy.loginfo("Turn angle: %d", angle)
                self.turn.action(angle)
                time.sleep(1)
                self.is_canceling = False
                self.is_voice = False
            r.sleep()

    def audio_cb(self, msg):
        if self.is_canceling is True:
            return

        self.audio_text = self.ai.audio_to_text(msg.data)	
        if self.audio_text is not None:
            rospy.loginfo("You Said: %s", self.audio_text.encode('utf-8'))
            rospy.loginfo("doa: %d", self.doa)
            self.is_voice = True
            self.is_canceling = True


    def direction_cb(self, msg):
        if self.is_canceling is False:
            self.doa = msg.data



if __name__ == '__main__':
    rospy.init_node("Abot_Summoner_Node")
    abotai = AbotSummoner()
    rospy.spin()
