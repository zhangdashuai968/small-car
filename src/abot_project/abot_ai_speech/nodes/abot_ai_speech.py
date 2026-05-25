#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
import os
import sys
from contextlib import contextmanager
import speech_recognition as SR
import requests
import pyaudio
import wave
import json
import time
from audio_common_msgs.msg import AudioData
from aip import AipSpeech
import base64
import uuid
from pyDes import *
import netifaces

error_reason={3300:      '输入参数不正确',
              3301:      '识别错误',
              3302:      '验证失败',
              3303:      '语音服务器后端问题',
              3304:      '请求 GPS 过大，超过限额',
              3305:      '产品线当前日请求数超过限额',
              3307:      '识别错误'}

@contextmanager
def ignore_stderr(enable=True):
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield



class AbotAISpeech(object):
    def __init__(self, device_id=0, audio_path=None):
        # format of input audio data
        self.device_id = device_id
        self.dir_path = audio_path

        self.app_id = '17067547'
        self.app_key = 'Lg2cXwFO3BB8aiOkw2pfVmNb'
        self.secret_key = 'GhoaLmguEYvT3422qye40XaL7PuiRLYV'
        self.client = AipSpeech(self.app_id, self.app_key, self.secret_key)

        # Turing API, replace with your personal key
        self.turing_key = "dd234b783ba14c3d8281ee88c04893b3"
        self.url= "http://openapi.tuling123.com/openapi/api/v2"
        self.header = {'Content-Type': 'application/json;charset=UTF-8'}

        self.recognizer = SR.Recognizer()


    def audio_to_text(self, audio_data, sample_rate=16000, sample_width=2):
        try:
            data = SR.AudioData(audio_data, sample_width, sample_width)
            result = self.client.asr(data.get_wav_data(), 'wav', 16000, {'dev_pid': 1536})
            if result[u'err_msg']=='success.':
                result_text = result["result"][0]
                return result_text
            else:
                return None
        except SR.UnknownValueError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))
        except SR.RequestError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))


    def speak_text(self, text=""):
        data = {"reqType": 0,
            "perception": {"inputText": {"text": ""},
            "selfInfo": {"location": {"city": "北京", "street": "中南海"}}},
            "userInfo": {"apiKey": self.turing_key, "userId": "rikibot"}
        }
        data["perception"]["inputText"]["text"] = text
        response = requests.request("post", self.url, json=data, headers=self.header)
        response_dict = json.loads(response.text)
        result = response_dict["results"][0]["values"]["text"]
        #rospy.loginfo("The AI said: " + result.encode('utf-8'))
        return result

    def text_to_audio(self, text=""):
        result = self.client.synthesis(text, 'zh', 1, {'spd': 4, 'vol': 5, 'per': 4,})
        if not isinstance(result, dict):
            with open(self.dir_path + 'audio.mp3', 'wb') as f:
                f.write(result)

        # Pyaudio to play mp3 file
    def audio_play(self):
        convert_audio = 'sox ' + self.dir_path + 'audio.mp3 ' + self.dir_path + 'audio.wav'
        os.system(convert_audio)
        wf = wave.open(self.dir_path  + 'audio.wav', 'rb')
        with ignore_stderr(enable=True):
            p = pyaudio.PyAudio()

        def callback(in_data, frame_count, time_info, status):
            data = wf.readframes(frame_count)
            return (data, pyaudio.paContinue)

        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
            channels=wf.getnchannels(),
            rate=wf.getframerate(),
            output=True,
            output_device_index = self.device_id,
            stream_callback=callback)

        stream.start_stream()
        while stream.is_active():
            time.sleep(0.1)

        stream.stop_stream()
        stream.close()
        wf.close()
        p.terminate()

