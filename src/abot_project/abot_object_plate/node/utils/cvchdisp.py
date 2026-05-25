#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2 as cv
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import os


class CvChDisp:
    def __init__(self):
        self.color = (255, 0, 0)
        self.encode = "utf-8"
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        print(self.dir_path)

        self.font_path = self.dir_path + "/msyh.ttf"

    def text(self, image, top, bottom):
        pilimg = Image.fromarray(image)
        draw = ImageDraw.Draw(pilimg)
        font = ImageFont.truetype(self.font_path, 30, encoding=self.encode)
        draw.text(top, bottom, self.color, font=font)
        cv2charimg = cv.cvtColor(np.array(pilimg), cv.COLOR_RGB2BGR)


        return cv2charimg

