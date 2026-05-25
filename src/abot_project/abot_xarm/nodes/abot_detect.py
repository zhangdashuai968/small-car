#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Authors: Rikibot kevin

import math
import numpy as np
import cv2


class ObjectDetection(object):
    def __init__(self):
	self.color_dist = {'red': {'Lower': np.array([16, 101, 171]), 'Upper': np.array([77, 255, 255])},
              #'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              #'yellow': {'Lower': np.array([20, 100, 100]), 'Upper': np.array([30, 255, 255])},
              #'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              } 

        #self.ball_color = 'red'

	#self.position_color_list = []
        self.last_blocks = []
        self.storage_blocks = []
        self.cv_blocks_ok = False
        self.c_angle = 0 
        self.stable = False
	self.cv_count = 0
	self.last_x = 0


    def detect(self, image):
        if self.cv_blocks_ok is False:
            # 高斯模糊
            gs_frame = cv2.GaussianBlur(image, (5, 5), 0)
            # 转换颜色空间
            hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
            position_color_list = []
            for i in self.color_dist:
                # 查找字典颜色
                mask = cv2.inRange(hsv, self.color_dist[i]['Lower'], self.color_dist[i]['Upper'])
                # 腐蚀
                mask = cv2.erode(mask, None, iterations=2)
                # 膨胀
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.dilate(mask, kernel, iterations=2)
                # 查找轮廓
                #cv2.imshow('mask', mask)
                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                if len(cnts) > 0:
                    #print("len: %d", len(cnts))
                    # 找出最大的区域
                    c = max(cnts, key=cv2.contourArea)
                    # 返回的值 中心坐标（x, y）,（w，h）,角度
                    rect = cv2.minAreaRect(c)
                    # 获取最小外接矩形的4个顶点
                    box = cv2.boxPoints(rect)
                    # 数据类型转换
                    # 绘制轮廓
                    cv2.drawContours(image, [np.int0(box)], -1, (0, 255, 255), 2)
                    # 找色块中心点
                    c_x, c_y = rect[0]
                    h, w = rect[1]
                    self.c_angle = rect[2]
                    #print(h*w)
                    if h * w >= 1000:   # 色块面积限制
                    # 绘制中心点
                        cv2.circle(image, (int(c_x), int(c_y)), 3, (216, 0, 255), -1)
                        self.last_blocks.append([int(c_x), i])
                        #print("color:", i)
                        if self.stable:
                            self.storage_blocks.append((int(c_y), int(c_x), i, int(self.c_angle)))
                 
            self.stable = False
            if len(self.last_blocks) > 0:
                #print(len(self.last_blocks))
                if -10 <= int(self.last_blocks[len(self.last_blocks) - 1][0] - self.last_x) <= 10:    # 只判读最后一个方块是否稳定
                    #print (self.cv_count)
                    self.cv_count += 1
                else:
                    self.cv_count = 0
                self.last_x = int(self.last_blocks[len(self.last_blocks) - 1][0])
                self.last_blocks = []
                if self.cv_count >= 5:
                    self.cv_count = 0
                    self.stable = True
                if len(self.storage_blocks) > 0:
                    max_y = self.storage_blocks.index(max(self.storage_blocks))
                    # 存储稳定后的数据， 颜色， X, Y, 色块角度
                    position_color_list.append((self.storage_blocks[max_y][2], self.storage_blocks[max_y][1],
                                                self.storage_blocks[max_y][0], self.storage_blocks[max_y][3]))
                    self.storage_blocks = []
                    #print("append list")
                    return position_color_list
                else:
                    return None

