#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import math
import numpy as np
 
class xArmKinematic:
    A1 = 81.5
    A2 = 73.15
    A3 = 73.15
    A4 = 210

    MAX_LEN = A2+A3+A4
    MAX_HIGH = A1+A2+A3+A4

    JOINT_ANGLE_DEFAULT = np.radians([0, -90.0, 90.0, 0, 0.0, 0.0])
    # 关节角度下限, 单位: 弧度
    JOINT_ANGLE_LOWERB = np.float64([-1.5708, -2.65, -1.5708, -2.0944, -1.5708])
    # 关节角度上限制, 单位: 弧度
    JOINT_ANGLE_UPPERB = np.float64([1.5708, 1.5708, 2.18166, 2.0944, 1.5708])

    def __init__(self, is_debug=False):
        '''初始化'''
        self.is_debug = is_debug
        # 当前关节角度赋值为默认角度
        self.cur_joint_angle = np.copy(self.JOINT_ANGLE_DEFAULT)

    def cos(self, degree):
        return math.cos(math.radians(degree))
     
    def sin(self, degree):
        return math.sin(math.radians(degree))
     
     
    def atan2(self, v1, v2):
        rad=math.atan2(v1, v2)
        return math.degrees(rad)

    def _j_degree_convert(self, joint,  j_or_deg):
        # 将j1-j4和机械臂的角度表达互换
        if joint == 1:
            res = j_or_deg
        elif joint == 2 or joint == 3 or joint == 4:
            res = 90 - j_or_deg
        else:
            # 只适用于1-4关节
            raise ValueError
        return res

     
    def _valid_degree(self, joint, degree):
        if 0 <= degree <= 180:
            return True 
        else:
            print('joint {} is invalid degree {}'.format(joint, degree))
            return False
     
    def _valid_j(self, joint, j):
        if j is None:
            return False

        degree = self._j_degree_convert(joint, j)
        if 0 <= degree <= 180:
            return True
        else:
            #print('joint {} is invalid j:{} degree {}'.format(joint, j, degree))
            return False
     
    def _out_of_range(self, lengh, height):
        if height > self.MAX_HIGH:
            #print('高度 {} 超过界限 {}'.format(height, self.MAX_HIGH))
            return True
        if lengh > self.MAX_LEN:
            #print('投影长度 {} 超过界限 {}'.format(lengh, self.MAX_LEN))
            return True
        return False
     
     
    def _calculate_j1(self, x, y, z):
        length = round(math.sqrt(pow((y), 2) + pow(x, 2)), 2)
        if length == 0:
            j1 = 0 #可以是任意数
        else:
            j1 = self.atan2((y),x)
        hight = z
        return j1, length, hight
     
    def _calculate_j3(self, L, H):
        cos3 = (L**2 + H**2 - self.A2**2 - self.A3**2)/(2*self.A2*self.A3)
        if (cos3**2>1):
            return None
        sin3 = math.sqrt(1 - cos3**2)
        j3 = self.atan2(sin3, cos3)
        return j3
     
     
    def _calculate_j2(self, L, H, j3):
        K1 = self.A2 + self.A3*self.cos(j3)
        K2 = self.A3*self.sin(j3)
        w = self.atan2(K2, K1)
        j2 = self.atan2(L, H) - w
        return j2
     
    def _calculate_j4(self, j2, j3, alpha):
        j4 = alpha - j2 - j3
        return j4
     
     
     
    def _xyz_alpha_to_j123(self, x, y, z, alpha):
        valid = False
        j1, j2, j3, j4 = None, None, None, None
        j1, length, height = self._calculate_j1(x, y, z)
        if self._valid_j(1, j1) and not self._out_of_range(length, height):
            L = length - self.A4 * self.sin(alpha)
            H = height - self.A4 * self.cos(alpha) - self.A1
            j3 = self._calculate_j3(L, H)
            if self._valid_j(3, j3):
                j2 = self._calculate_j2(L, H, j3)
                if self._valid_j(2,j2):
                    j4 = self._calculate_j4(j2, j3, alpha)
                    if self._valid_j(4, j4):
                        valid = True
        return valid, j1, j2, j3, j4
     
     
    def _xyz_to_j123(self, x,y,z, alpha=180):
        MIN_ALPHA = 90 # j2+j3+j4 min value, 最后一个joint不向后仰
        valid = False
        while alpha >= MIN_ALPHA and not valid:
            valid, j1, j2, j3, j4 = self._xyz_alpha_to_j123(x, y, z, alpha)
            if not valid:
                alpha -= 1
        return valid, j1, j2, j3, j4
     
     
     
    def inverse_kinematics(self, x, y, z, alpha=180):
        x=float(x)
        y=float(y)
        z=float(z)
        #print('x:{} y:{} z:{} alpha:{}'.format(x,y,z,alpha))
        valid, j1, j2, j3, j4 = self._xyz_to_j123(y, x, z, alpha)
        if valid:
            deg1 = round(self._j_degree_convert(1, j1), 2)
            deg2 = round(self._j_degree_convert(2, j2), 2)
            deg3 = round(self._j_degree_convert(3, j3), 2)
            deg4 = round(self._j_degree_convert(4, j4), 2)

        print('valid:{},deg1:{},deg2:{},deg3:{},deg4:{}'.format(valid, deg1, deg2, deg3, deg4))
        print('[{},{},{},{}]'.format(np.deg2rad(deg1), np.deg2rad(deg2), np.deg2rad(deg3), np.deg2rad(deg4)))
        print('[{},{},{},{}]'.format(np.deg2rad(deg1) - np.deg2rad(90), -np.deg2rad(deg2)-np.deg2rad(45), np.deg2rad(45)+np.deg2rad(deg3), np.deg2rad(deg4) + np.deg2rad(90)))
     
        return valid, deg1, deg2, deg3, deg4
     
     
    def forward_kinematics(self, deg1, deg2, deg3, deg4):
        j1=self._j_degree_convert(1, deg1)
        j2=self._j_degree_convert(2, deg2)
        j3=self._j_degree_convert(3, deg3)
        j4=self._j_degree_convert(4, deg4)
        print('j1:{},j2:{},j3:{},j4:{}'.format(j1, j2, j3, j4))
        length = self.A2*self.sin(j2) + self.A3*self.sin(j2+j3) + self.A4*self.sin(j2+j3+j4)
        height = self.A1 + self.A2*self.cos(j2) + self.A3*self.cos(j2+j3) + self.A4*self.cos(j2+j3+j4)
        alpha = j2 + j3 + j4
     
        z = round(height, 2)
        y = round(length*self.cos(j1))
        x = round(length*self.sin(j1))
      
     
        print('x:{},y:{},z:{},lenghth:{},height:{},alpha:{}'.format(x, y, z, round(length,2), round(height,2), alpha))
     
        return valid, x, y, z
 

if __name__ == '__main__':
    x = 300
    y = 0
    z = 30
    arm = xArmKinematic()
    valid, deg1, deg2, deg3, deg4=arm.inverse_kinematics(x, y, z, alpha=180)
    if valid:
        valid,x1,y1,z1=arm.forward_kinematics(deg1, deg2, deg3, deg4)
        if abs(x1-x) > 5 or abs(y1-y) > 5 or abs(z1-z) > 5:
            print('err')
        else:
            print('ok')

    print('done')

