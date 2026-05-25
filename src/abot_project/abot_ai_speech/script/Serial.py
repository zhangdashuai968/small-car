#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: kevin <304050118@qq.com>

import os
import sys
import uuid
import netifaces


def Serail():
    mac = netifaces.ifaddresses('eth0')[netifaces.AF_LINK]
    return mac[0]['addr']

print(Serail())



