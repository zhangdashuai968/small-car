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


app_id = '17067547'
app_key = 'Lg2cXwFO3BB8aiOkw2pfVmNb'
secret_key = 'GhoaLmguEYvT3422qye40XaL7PuiRLYV'
client = AipSpeech(app_id, app_key, secret_key)

text = "任务完成"
result = client.synthesis(text, 'zh', 1, {'spd': 4, 'vol': 5, 'per': 4,})
if not isinstance(result, dict):
    with open('audio.mp3', 'wb') as f:
        f.write(result)




