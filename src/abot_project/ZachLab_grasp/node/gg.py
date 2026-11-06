#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
'''

import rospy
import time
import json
from xarm_driver import *
from vl_locate.msg import Detection,DetectionArray
import numpy as np

class BasicXArmGrasp():
    def __init__(self):
        # 从参数服务器获取机械臂位置参数
        self.x = rospy.get_param("~x", 0)
        self.y = rospy.get_param("~y", 0) 
        self.z = rospy.get_param("~z", 250)
        # M1-S实测home(2026-11-01标定)：独立保存，防抓取流程覆盖
        self.home_x = rospy.get_param("~home_x", 45)
        self.home_y = rospy.get_param("~home_y", -5)
        self.home_z = rospy.get_param("~home_z", 230)
        self.rotary = rospy.get_param("~rotary", 90)
        
        rospy.set_param('grab', False) 
        # 从参数服务器获取控制参数
        self.speed = rospy.get_param("~speed", 1000)  # 舵机控制速度 ms
        self.port = rospy.get_param("~port", '/dev/ttyACM0')
        self.baud = rospy.get_param("~baud", 115200)
        
        # 从参数服务器获取夹爪位置参数
        self.gripper_close = rospy.get_param("~gripper_close", 700)
        self.gripper_open = rospy.get_param("~gripper_open", 150)
        self.gripper_default = rospy.get_param("~gripper_default", 500)
        
        # 从参数服务器获取标定参数
        self.cali_1_im = rospy.get_param("~cali_1_im", [66, 361])  # 左下角标定点像素坐标
        self.cali_1_mc = rospy.get_param("~cali_1_mc", [-60, 125])  # 左下角标定点机械臂坐标
        self.cali_2_im = rospy.get_param("~cali_2_im", [584, 125])  # 右上角标定点像素坐标
        self.cali_2_mc = rospy.get_param("~cali_2_mc", [135, 195])  # 右上角标定点机械臂坐标
        # 可选第三锚点(11-02三点标定升级)：两点线性在对角线外象限有残差
        self.cali_3_im = rospy.get_param("~cali_3_im", None)
        self.cali_3_mc = rospy.get_param("~cali_3_mc", None)
        self.cali_4_im = rospy.get_param("~cali_4_im", None)
        self.cali_4_mc = rospy.get_param("~cali_4_mc", None)
        self.cali_5_im = rospy.get_param("~cali_5_im", None)
        self.cali_5_mc = rospy.get_param("~cali_5_mc", None)
        self.cali_6_im = rospy.get_param("~cali_6_im", None)  # 11-06第6锚:中心↔左下带
        self.cali_6_mc = rospy.get_param("~cali_6_mc", None)
        self.cali_7_im = rospy.get_param("~cali_7_im", None)  # 11-06第7锚:中心右下带
        self.cali_7_mc = rospy.get_param("~cali_7_mc", None)
        # 11-02四点标定: 转盘角度随像素y插值(四角实测40~120各不相同,单一factor罩不住)
        self.cali_rotary_anchors = rospy.get_param("~cali_rotary_anchors", None)
        
        # 从参数服务器获取抓取参数
        self.grasp_z = rospy.get_param("~grasp_z", -110)  # 抓取高度
        self.grasp_offset_x = rospy.get_param("~grasp_offset_x", 10)  # X轴偏移
        self.grasp_offset_y = rospy.get_param("~grasp_offset_y", 0)  # X轴偏移
        self.place_offset_x = rospy.get_param("~place_offset_x", 0)  # 放置专用X轴偏移
        self.place_offset_y = rospy.get_param("~place_offset_y", 0)  # 放置专用Y轴偏移
        self.rotation_factor = rospy.get_param("~rotation_factor", 0.33)  # 旋转因子
        self.max_rotation_angle = rospy.get_param("~max_rotation_angle", 45)  # 最大旋转角度
        
        # 从参数服务器获取模式参数
        self.continuous_mode = rospy.get_param("~continuous_mode", True)  # True为持续抓取，False为单次抓取
        
        # 从参数服务器获取话题名称
        self.pose_topic = rospy.get_param("~pose_topic", '/Pose')
        
        # 订阅物体信息话题
        rospy.Subscriber(self.pose_topic, DetectionArray, self.pose_callback)
        
        # 初始化机械臂控制器
        self.xarm_ctrl = xArmCtrl(port=self.port, baud=self.baud)

        rospy.set_param('success', False)
        
        # 状态变量
        self.grasp_obj = None
        self.obj_detected = False
        
        # 机械臂回到初始位置
        self.xarm_home(self.gripper_open)
        
        rospy.loginfo("机械臂基础抓取系统初始化完成")
        rospy.loginfo("运行模式: %s", "持续抓取" if self.continuous_mode else "单次抓取")

    def pose_callback(self, object_msg):
        """物体信息回调函数"""
        if len(object_msg.detections) > 0:
            # 获取第一个检测到的物体
            self.grasp_obj = object_msg.detections[0]
            self.obj_detected = True
            rospy.loginfo("检测到物体: x=%d, y=%d, z=%d", 
                         self.grasp_obj.x_center, 
                         self.grasp_obj.y_center, 
                         self.grasp_z)

    def camera_point_to_grasp_pose(self, obj_info):
        """相机坐标转换为抓取位姿"""
        
        # 11-06仿射最小二乘拟合(替代IDW:IDW锚间波浪形吸偏;仿射全平面平滑,残差RMS约1.1cm)
        # 2锚=对角线性插值;>=3锚=np.linalg.lstsq拟合 X_mc=a*px+b*py+c
        anchors = []
        for im, mc in [(self.cali_1_im, self.cali_1_mc),
                       (self.cali_2_im, self.cali_2_mc),
                       (self.cali_3_im, self.cali_3_mc),
                       (self.cali_4_im, self.cali_4_mc),
                       (self.cali_5_im, self.cali_5_mc),
                       (self.cali_6_im, self.cali_6_mc),
                       (self.cali_7_im, self.cali_7_mc)]:
            if im and mc:
                anchors.append((float(im[0]), float(im[1]), float(mc[0]), float(mc[1])))
        px, py = float(obj_info.x_center), float(obj_info.y_center)

        # 11-07安全区护栏: 像素超出锚点覆盖区(±25px余量)直接放弃,防仿射外推甩臂
        if anchors:
            xs = [a[0] for a in anchors]
            ys = [a[1] for a in anchors]
            if not (min(xs) - 25 <= px <= max(xs) + 25 and
                    min(ys) - 25 <= py <= max(ys) + 25):
                rospy.logwarn("检测像素(%d,%d)超出标定安全区,放弃本次动作", int(px), int(py))
                self.pose_valid = False
                return
        self.pose_valid = True

        if len(anchors) == 2:
            x_ims = [anchors[0][0], anchors[1][0]]
            x_mcs = [anchors[0][2], anchors[1][2]]
            y_ims = [anchors[0][1], anchors[1][1]]
            y_mcs = [anchors[0][3], anchors[1][3]]
            xo = np.argsort(x_ims)
            yo = np.argsort(y_ims)
            X = int(round(np.interp(px, [x_ims[i] for i in xo], [x_mcs[i] for i in xo])))
            Y = int(round(np.interp(py, [y_ims[i] for i in yo], [y_mcs[i] for i in yo])))
        else:
            Mtrx = np.array([[a[0], a[1], 1.0] for a in anchors])
            bx = np.array([a[2] for a in anchors])
            by = np.array([a[3] for a in anchors])
            coeff_x, _, _, _ = np.linalg.lstsq(Mtrx, bx, rcond=-1)
            coeff_y, _, _, _ = np.linalg.lstsq(Mtrx, by, rcond=-1)
            X = int(round(coeff_x[0] * px + coeff_x[1] * py + coeff_x[2]))
            Y = int(round(coeff_y[0] * px + coeff_y[1] * py + coeff_y[2]))
        self.x = Y + self.grasp_offset_x
        self.y = -X+self.grasp_offset_y
        self.z = self.grasp_z

        if self.cali_rotary_anchors:
            rp = sorted([tuple(a) for a in self.cali_rotary_anchors])
            self.rotary = int(np.interp(obj_info.x_center,
                                        [t[0] for t in rp], [t[1] for t in rp]))
        elif self.y > 0:
            rotation_angle = min(abs(self.y) * self.rotation_factor, self.max_rotation_angle)
            self.rotary = 90 + rotation_angle
        elif self.y < 0:
            rotation_angle = min(abs(self.y) * self.rotation_factor, self.max_rotation_angle)
            self.rotary = 90 - rotation_angle
        else:
            self.rotary = 90
        rospy.loginfo("抓取位姿: x=%d, y=%d, z=%d", self.x, self.y, self.z)

    def xarm_pose_tx_data(self, x, y, z, rotary=90):
        """发送机械臂位姿数据"""
        xarm_pose = {
            "RikibotxArmCmd": 1,
            "Pose": {"x": x, "y": y, "z": z, "rotary": rotary},
            "time": self.speed
        }
        self.xarm_ctrl.SerialWriteData(json.dumps(xarm_pose))

    def xarm_gripper_tx_data(self, gripper_step):
        """发送夹爪控制数据"""
        xarm_gripper = {
            "RikibotxArmCmd": 2, 
            "Gripper": gripper_step, 
            "time": self.speed
        }
        self.xarm_ctrl.SerialWriteData(json.dumps(xarm_gripper))

    def xarm_home(self, gripper_step=None):
        """机械臂回到初始位置"""
        if gripper_step is None:
            gripper_step = self.gripper_close
        self.xarm_pose_tx_data(self.home_x, self.home_y, self.home_z)
        time.sleep(1)
        self.xarm_gripper_tx_data(gripper_step)
        rospy.loginfo("机械臂回到初始位置")

    def grasp_object(self):
        """抓取物体"""
        if not self.obj_detected or self.grasp_obj is None:
            rospy.logwarn("未检测到物体，无法执行抓取")
            return False
            
        rospy.loginfo("开始抓取物体...")
        
        # 1. 打开夹爪
        self.xarm_gripper_tx_data(self.gripper_open)
        time.sleep(1)
        
        # 2. 计算抓取位姿
        self.camera_point_to_grasp_pose(self.grasp_obj)
        if not getattr(self, 'pose_valid', True):
            rospy.logwarn("抓取目标超出安全区,清状态放行主控继续跑")
            self.obj_detected = False
            self.grasp_obj = None
            rospy.set_param('success', True)
            return False
        time.sleep(1)
        
        # 3. 移动到抓取位置
        self.xarm_pose_tx_data(self.x, self.y, self.z, self.rotary)
        time.sleep(2)
        
        # 4. 闭合夹爪
        self.xarm_gripper_tx_data(self.gripper_close)
        time.sleep(2)
        
        # 5. 回到初始位置
        self.xarm_home(self.gripper_close)
        time.sleep(2)

        # self.xarm_gripper_tx_data(self.gripper_open)
        # time.sleep(1)
        
        # 重置状态
        self.obj_detected = False
        self.grasp_obj = None
        self.grasp= False  # 重置标志位
        rospy.set_param('/grab', False)
        rospy.loginfo("抓取完成！")
        
        rospy.set_param('success', True)
        print("Setting 'success' to True.")
        return True

    def put_object(self):
        """put物体"""
        if not self.obj_detected or self.grasp_obj is None:
            rospy.logwarn("未检测到物体，无法执行抓取")
            return False
            
        rospy.loginfo("开始put物体...")

        # 11-07: 放置保持抓取时的转盘角,防止带着物体旋转甩飞
        keep_rotary = self.rotary
        # 1. 计算put位姿
        self.camera_point_to_grasp_pose(self.grasp_obj)
        if not getattr(self, 'pose_valid', True):
            rospy.logwarn("放置目标超出安全区,清状态放行主控继续跑")
            self.obj_detected = False
            self.grasp_obj = None
            rospy.set_param('success', True)
            return False
        self.rotary = keep_rotary
        self.x += self.place_offset_x
        self.y += self.place_offset_y
        rospy.loginfo("放置补偿后位姿: x=%d, y=%d, z=%d",
                      self.x, self.y, -70)
        time.sleep(1)
        
        # 2. 移动到put位置
        self.xarm_pose_tx_data(self.x, self.y, -70, self.rotary)
        time.sleep(2)
        # 3. 闭合夹爪
        self.xarm_gripper_tx_data(self.gripper_open)
        time.sleep(2)
        
        # 4. 回到初始位置
        self.xarm_home(self.gripper_close)
        time.sleep(2)

        # 重置状态
        self.obj_detected = False
        self.grasp_obj = None
        rospy.loginfo("put完成！")
        
        rospy.set_param('success', True)
        print("Setting 'success' to True.")
        return True

    def wait_for_object_and_grasp(self):
        """等待检测到物体并执行抓取"""
        rospy.loginfo("等待检测物体...")
        
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.grasp=rospy.get_param('/grab', False)
            if self.obj_detected:
                if self.grasp:
                    self.grasp_object()
                    # 抓取完成后继续等待下一个物体
                    rospy.loginfo("等待下一个物体...")
                else:
                    self.put_object()
                    rospy.loginfo("等待下一个物体...")
            rate.sleep()

    def single_grasp(self):
        """执行单次抓取（如果检测到物体）"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.obj_detected:
                self.grasp_object()
                # 抓取完成后继续等待下一个物体
                rospy.loginfo("抓取完成")
                break
            rate.sleep()

    def run(self):
        """根据模式参数运行相应的抓取模式"""
        if self.continuous_mode:
            # 持续抓取模式
            self.wait_for_object_and_grasp()
        else:
            # 单次抓取模式
            rospy.loginfo("单次抓取模式，等待物体检测...")
            self.single_grasp()
            rospy.spin()

    def shutdown(self):
        """关闭机械臂"""
        rospy.logwarn("正在关闭机械臂...")
        self.xarm_home(self.gripper_open)


def main():
    rospy.init_node('basic_xarm_grasp')
    
    # 创建机械臂抓取对象
    xarm_grasp = BasicXArmGrasp()
    
    # 设置关闭回调
    rospy.on_shutdown(xarm_grasp.shutdown)
    
    try:
        # 根据参数运行相应模式
        xarm_grasp.run()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()