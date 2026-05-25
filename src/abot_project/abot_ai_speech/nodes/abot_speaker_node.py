#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: kevin <304050118@qq.com>

import angles
from contextlib import contextmanager
import usb.core
import usb.util
import pyaudio
import math
import numpy as np
import tf.transformations as T
import os
import rospy
import struct
import sys
import time
from audio_common_msgs.msg import AudioData
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32, ColorRGBA
from abot_speaker_driver import AbotSpeakerDriver, AbotSpeakerAudio



class AbotSpeakerNode(object):
    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)
        self.update_rate = rospy.get_param("~update_rate", 10.0)
        self.sensor_frame_id = rospy.get_param("~sensor_frame_id", "respeaker_base")
        self.doa_xy_offset = rospy.get_param("~doa_xy_offset", 0.0)
        self.doa_yaw_offset = rospy.get_param("~doa_yaw_offset", 90.0)
        self.speech_prefetch = rospy.get_param("~speech_prefetch", 0.5)
        self.speech_continuation = rospy.get_param("~speech_continuation", 0.5)
        self.speech_max_duration = rospy.get_param("~speech_max_duration", 7.0)
        self.speech_min_duration = rospy.get_param("~speech_min_duration", 0.1)
        self.main_channel = rospy.get_param('~main_channel', 0)
        suppress_pyaudio_error = rospy.get_param("~suppress_pyaudio_error", True)
        #
        self.speaker = AbotSpeakerDriver()
        self.speaker_audio = AbotSpeakerAudio(self.on_audio, suppress_error=suppress_pyaudio_error)
        self.speech_audio_buffer = str()
        self.is_speeching = False
        self.speech_stopped = rospy.Time(0)
        self.prev_is_voice = None
        self.prev_doa = None
        # advertise
        self.pub_vad = rospy.Publisher("is_speeching", Bool, queue_size=1, latch=True)
        self.pub_doa_raw = rospy.Publisher("sound_direction", Int32, queue_size=1, latch=True)
        self.pub_doa = rospy.Publisher("sound_localization", PoseStamped, queue_size=1, latch=True)
        self.pub_audio = rospy.Publisher("audio", AudioData, queue_size=10)
        self.pub_speech_audio = rospy.Publisher("speech_audio", AudioData, queue_size=10)
        self.pub_audios = {c:rospy.Publisher('audio/channel%d' % c, AudioData, queue_size=10) for c in self.speaker_audio.channels}

        # start
        self.speech_prefetch_bytes = int(
            self.speech_prefetch * self.speaker_audio.rate * self.speaker_audio.bitdepth / 8.0)
        self.speech_prefetch_buffer = str()
        self.speaker_audio.start()
        self.info_timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate),
                                      self.on_timer)
        self.timer_led = None

    def on_shutdown(self):
        try:
            self.speaker.close()
        except:
            pass
        finally:
            self.speaker = None
        try:
            self.speaker_audio.stop()
        except:
            pass
        finally:
            self.speaker_audio = None


    def on_audio(self, data, channel):
        self.pub_audios[channel].publish(AudioData(data=data))
        if channel == self.main_channel:
            self.pub_audio.publish(AudioData(data=data))
            if self.is_speeching:
                if len(self.speech_audio_buffer) == 0:
                    self.speech_audio_buffer = self.speech_prefetch_buffer
                self.speech_audio_buffer += data
            else:
                self.speech_prefetch_buffer += data
                self.speech_prefetch_buffer = self.speech_prefetch_buffer[-self.speech_prefetch_bytes:]

    def on_timer(self, event):
        stamp = event.current_real or rospy.Time.now()
        is_voice = self.speaker.is_voice()
        doa_rad = math.radians(self.speaker.direction - 180.0)
        doa_rad = angles.shortest_angular_distance(
            doa_rad, math.radians(self.doa_yaw_offset))
        doa = math.degrees(doa_rad)

        # vad
        if is_voice != self.prev_is_voice:
            self.pub_vad.publish(Bool(data=is_voice))
            self.prev_is_voice = is_voice

        # doa
        if doa != self.prev_doa:
            self.pub_doa_raw.publish(Int32(data=doa))
            self.prev_doa = doa

            msg = PoseStamped()
            msg.header.frame_id = self.sensor_frame_id
            msg.header.stamp = stamp
            ori = T.quaternion_from_euler(math.radians(doa), 0, 0)
            msg.pose.position.x = self.doa_xy_offset * np.cos(doa_rad)
            msg.pose.position.y = self.doa_xy_offset * np.sin(doa_rad)
            msg.pose.orientation.w = ori[0]
            msg.pose.orientation.x = ori[1]
            msg.pose.orientation.y = ori[2]
            msg.pose.orientation.z = ori[3]
            self.pub_doa.publish(msg)

        # speech audio
        if is_voice:
            self.speech_stopped = stamp
        if stamp - self.speech_stopped < rospy.Duration(self.speech_continuation):
            self.is_speeching = True
        elif self.is_speeching:
            buf = self.speech_audio_buffer
            self.speech_audio_buffer = str()
            self.is_speeching = False
            duration = 8.0 * len(buf) * self.speaker_audio.bitwidth
            duration = duration / self.speaker_audio.rate / self.speaker_audio.bitdepth
            rospy.loginfo("Abot Speech detected for %.3f seconds" % duration)
            if self.speech_min_duration <= duration < self.speech_max_duration:
                self.pub_speech_audio.publish(AudioData(data=buf))


if __name__ == '__main__':
    rospy.init_node("Abot_Speaker_Node")
    n = AbotSpeakerNode()
    rospy.spin()
