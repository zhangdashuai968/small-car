#!/usr/bin/env python2
# _*_ coding:utf-8 _*_
# 机器人机械臂抓取控制节点
# 该节点负责控制xArm机械臂进行物体抓取和放置操作
# 包括导航到目标位置、视觉识别物体、控制机械臂抓取和放置等功能

import rospy
import os, math, time, json
import numpy as np
import time
import cv2
from tf.transformations import quaternion_from_euler
from std_msgs.msg import UInt8, Float64, Bool
from dynamic_reconfigure.server import Server
from abot_xarm.cfg import XarmPoseParamsConfig
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from abot_grasp_race.msg import ObjectInfo, ObjectInfoArray
from xarm_driver import *
from nav import *
from geometry_msgs.msg import Twist


# 机械臂夹爪状态定义
GRIPPER_CLOSE = 700    # 夹爪闭合状态值
GRIPPER_OPEN = 150     # 夹爪张开状态值
GRIPPER_DEFAULT = 500  # 夹爪默认状态值

class AbotxArmGrasp():
    """
    Abot xArm机械臂控制类
    该类负责初始化机械臂参数、订阅ROS话题、发布控制指令，并管理抓取任务的各个阶段
    """
    def __init__(self):
        """
        初始化AbotxArmGrasp类
        设置机械臂参数、导航参数、校准数据等，并初始化ROS订阅者和发布者
        """
        # 机械臂位置参数
        self.x = rospy.get_param("~x", 0)        # 机械臂X轴坐标
        self.y = rospy.get_param("~y", 0)        # 机械臂Y轴坐标
        self.z = rospy.get_param("~z", 250)      # 机械臂Z轴坐标
        self.rotary = rospy.get_param("~rotary", 90)  # 机械臂旋转角度

        # 机械臂控制参数
        self.speed = rospy.get_param("~speed", 1000)  # 机械臂控制速度，单位ms
        self.port = rospy.get_param("~port", '/dev/ttyACM0')  # 串口端口
        self.baud = rospy.get_param("~baud", 115200)  # 串口波特率

        # 模式控制参数
        self.fix_grasp_mode = rospy.get_param("~fix_grasp_mode", True)  # 固定抓取模式
        self.is_calibration_mode = rospy.get_param("~calibration_mode", True)  # 校准模式
        self.calib_data = rospy.get_param("~calib_data", [55, -58, 513, 190, -65, -30])  # 校准数据 [x1, y1, z1, x2, y2, z2]

        # 伺服控制参数
        self.servo_run = rospy.get_param("~servo_run", False)  # 是否运行伺服
        self.servo_x = rospy.get_param("~servo_x", 10)  # 伺服X轴参数
        
        # 抓取参数
        self.grasp_order = rospy.get_param("~grasp_order", ["red", "green", "yellow"])  # 抓取顺序
        self.place_offse_z = rospy.get_param("~place_offse_z", 30)  # 放置时Z轴偏移
        self.nav_offset = rospy.get_param("~nav_offset", 0.4)  # 导航偏移
        self.ky = rospy.get_param("~ky", 10)  # Y轴控制系数
        self.ctrl_time = rospy.get_param("~ctrl_time", 150)  # 控制时间
        self.obj_offset_x = rospy.get_param("~obj_offset_x", 30)  # X轴对象偏移
        self.obj_offset_y = rospy.get_param("~obj_offset_y", 50)  # Y轴对象偏移

        # 位置参数
        self.grasp_pos = rospy.get_param("~grasp_pos")  # 抓取位置
        self.place_pos = rospy.get_param("~place_pos")  # 放置位置
        self.end_pos = rospy.get_param("~end_pos")  # 结束位置

        # 配置文件路径
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.dir_path = self.dir_path.replace('abot_grasp_race/node', 'abot_grasp_race/')
        self.dir_path += 'config/'

        # 伺服控制
        if self.servo_run is True:
            os.system("rostopic pub -1 servo riki_msgs/Servo -- '90' '%d'" %(self.servo_x))

        # 初始化ROS订阅者和发布者
        rospy.Subscriber("/object_info", ObjectInfoArray, self.ObjectCallback)  # 订阅对象信息
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 发布速度控制指令
        self.grab_finished_pub = rospy.Publisher("/grab_finished", Bool, queue_size=1)  # 发布抓取完成状态
        rospy.Subscriber("/grab", Bool, self.grab_callback)  # 订阅抓取命令

        self.last_x = self.last_y = self.last_z = self.last_rotary = 0
        self.grasp_lock = False
        self.grasp_goal_lock = False
        self.grasp_finish = False

        self.place_lock = False
        self.place_goal_lock = False

        self.move_lock = False
        self.move_goal_lock = False

        self.calib_lock = False

        self.obj_cnt = 0
        self.task_cnt = 0
        self.nav_step = 0
        self.obj_ok = 20
        self.object_info = None
        self.grasp_obj = None
        self.grab_cmd = False

        self.kx = 0.2


        self.offset_x = self.calib_data[3] + self.calib_data[1] 
        self.offset_y = self.calib_data[4] + self.calib_data[0]
        self.offset_z = self.calib_data[2]  + self.calib_data[5]
        rospy.loginfo("offset value:x: %d, y: %d, z: %d", self.offset_x, self.offset_y, self.offset_z)


        r = rospy.Rate(5)
        rospy.on_shutdown(self.shutdown)

        self.xArmCtrl = xArmCtrl(port=self.port, baud=self.baud)

        if self.is_calibration_mode is True:
            self.xArmGripperTxData(GRIPPER_OPEN)
            srv_xarm = Server(XarmPoseParamsConfig, self.cbGetxArmPoseParam)

       
        if self.fix_grasp_mode is False and self.is_calibration_mode is False:
            rospy.loginfo("start navigation move to grasp pos")
            self.nav = Nav()

        if self.is_calibration_mode is False:
            self.xArmHome(GRIPPER_OPEN)

    def grab_callback(self, msg):
        """
        ROS订阅回调函数，用于处理来自'/grab'主题的抓取命令
        
        参数:
            msg (std_msgs.msg.Bool): 包含抓取命令的布尔消息
        """
        if msg.data is True:
            self.grab_cmd = True
            rospy.loginfo("Received grab command")

        while not rospy.is_shutdown():
            if self.is_calibration_mode is True:
                if self.last_x != self.x or self.last_y != self.y or self.last_z != self.z or self.last_rotary != self.rotary:
                    rospy.loginfo("Update Pose")
                    self.last_x = self.x
                    self.last_y = self.y
                    self.last_z = self.z
                    self.last_rotary = self.rotary
        
                    if self.grasp_lock is True:
                        obj_point = [int(self.grasp_obj.point_x), 
                                int(self.grasp_obj.point_y), 
                                int(self.grasp_obj.point_z)]
                        rospy.loginfo("current object pose: %d, %d, %d, current arm pose: %d, %d, %d", 
                                obj_point[0], obj_point[1], obj_point[2], self.x, self.y, self.z)
                    self.xArmPoseTxData(self.x, self.y, self.z, self.rotary)
            else:
                if self.fix_grasp_mode is False:
                    if self.grasp_goal_lock is True :
                        if self.grasp_lock is True:
                            rospy.loginfo("start grasp object!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            self.xArmGraspPlan(self.grasp_obj)
                    elif self.grab_cmd is True:
                        if self.grasp_lock is True:
                            rospy.loginfo("start grasp object by command!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            self.xArmGraspPlan(self.grasp_obj)
                            self.grab_cmd = False
                    elif self.move_goal_lock is True:
                        if self.move_lock is True:
                            rospy.loginfo("start move car to  object!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            self.MoveBackCar(self.grasp_obj)
                    else:
                        if self.task_cnt < len(self.grasp_order) and self.place_goal_lock is False:
                            rospy.loginfo("start navigtion and move to grasp object nav step %d", self.nav_step)
                            self.MoveGrasp()
                        if self.task_cnt == 3:
                            rospy.loginfo("start end pos")
                            self.task_cnt = self.task_cnt + 1
                            self.nav.SendGoal(self.NavPoseConvert(self.end_pos, 0, 0)) 
                    if self.place_goal_lock is True and self.place_lock is True:
                        rospy.loginfo("start place object")
                        self.xArmPlacePlan(self.grasp_obj)
                else:
                    if self.grasp_lock is True :
                        rospy.loginfo("Start Fix Grasp Object")
                        self.xArmFixGraspTest(self.grasp_obj)

            r.sleep()

    def ObjectCallback(self, object_msg):
        """
        ROS订阅回调函数，用于处理来自'/object_info'主题的对象信息
        根据当前模式（校准模式、固定抓取模式、移动抓取模式）执行相应的操作
        
        参数:
            object_msg (abot_grasp_race.msg.ObjectInfoArray): 包含检测到对象信息的数组消息
        """
        self.object_info = object_msg
        if self.fix_grasp_mode is True or self.grasp_goal_lock is True or self.place_goal_lock is True or self.move_goal_lock is True:
            self.obj_cnt = self.obj_cnt + 1

        if self.obj_cnt == self.obj_ok :
            if self.grasp_lock is False or self.place_lock is False or self.move_lock is False:
                for obj in self.object_info.detections:
                    if obj.obj_id == self.grasp_order[self.task_cnt]:
                        if self.grasp_goal_lock is True:
                            self.grasp_lock = True
                        if self.place_goal_lock is True:
                            self.place_lock = True
                        if self.move_goal_lock is True:
                            self.move_lock = True
                        self.grasp_obj = obj
                    else:
                        self.obj_cnt = 0

            if self.fix_grasp_mode is True and self.grasp_lock is False:
                self.grasp_obj = self.object_info.detections[-1]
                self.grasp_lock = True


    def CameraiPointToGraspPose(self, obj_info):
        """
        将相机检测到的对象点转换为机械臂的抓取姿态
        通过校准数据计算机械臂应移动到的精确位置
        
        参数:
            obj_info (abot_grasp_race.msg.ObjectInfo): 包含对象在相机坐标系中位置信息的对象
        """
        
        self.x = self.offset_x - obj_info.point_y + abs(self.calib_data[2]-obj_info.point_z)*self.kx
        if obj_info.point_x > self.obj_offset_y: 
            self.y = self.offset_y - obj_info.point_x -self.ky
        else:
            self.y = self.offset_y - obj_info.point_x 
        #self.z = self.offset_z - obj_info.point_z
        self.z = self.calib_data[5]
        rospy.loginfo("current obj x: %d, y: %d, z: %d, grasp pose: x: %d, y: %d, z: %d", 
                obj_info.point_x, obj_info.point_y, obj_info.point_z, self.x, self.y, self.z)


    def MoveGrasp(self):
        """
        控制机器人进行导航，移动到抓取或放置位置
        根据导航步骤发送不同的导航目标，并处理导航状态
        """
        if self.nav_step == 0:
            rospy.loginfo("first step")
            self.nav.SendGoal(self.NavPoseConvert(self.grasp_pos[self.task_cnt], 0, self.nav_offset))
        if self.nav_step == 1 and self.grasp_finish is True:
            rospy.loginfo("second step %d", self.task_cnt)
            self.MoveCar()
            self.nav.SendGoal(self.NavPoseConvert(self.place_pos[self.task_cnt], -self.nav_offset, 0))

        state =self.nav.NavState()
        if state == 1:
            self.nav_step = self.nav_step + 1

            if self.nav_step == 1:
                self.move_goal_lock = True
                self.obj_cnt = 0
            if self.nav_step == 2:
                self.move_goal_lock = True
                self.obj_cnt = 0

    def MoveBackCar(self, obj):
        """
        根据对象的位置调整机器人姿态，使其能够更好地抓取或放置
        通过发布Twist消息控制机器人的线速度和角速度
        
        参数:
            obj (abot_grasp_race.msg.ObjectInfo): 包含对象位置信息的对象
        """
        time.sleep(0.5)
        carmove = Twist()
        print(obj)
        if obj.point_y < -self.obj_offset_y:
            t = abs(obj.point_y)/self.ctrl_time
            print(t)
            carmove.linear.x = -0.2
            carmove.linear.y = 0
            carmove.angular.z = 0
            for i in range(1, 30):
                self.cmdvel_pub.publish(carmove)
            time.sleep(t)
        if obj.point_x > self.obj_offset_x or obj.point_x < -self.obj_offset_x:
            t = abs(obj.point_x)/self.ctrl_time
            print(t)
            carmove.linear.x = 0
            carmove.linear.y = 0.2*(abs(obj.point_x)/obj.point_x)
            carmove.angular.z = 0
            for i in range(1, 30):
                self.cmdvel_pub.publish(carmove)
            time.sleep(t)
        carmove.linear.x = 0
        carmove.linear.y = 0
        carmove.angular.z = 0
        for i in range(1, 30):
            self.cmdvel_pub.publish(carmove)
        self.move_lock = False
        self.move_goal_lock = False
        self.obj_cnt = 0
        if self.nav_step == 1:
            self.grasp_goal_lock = True
        if self.nav_step == 2:
            self.place_goal_lock = True

    def MoveCar(self):
        """
        控制机器人进行短距离移动，通常用于调整抓取前的姿态
        
        参数:
            无
        """
        time.sleep(0.5)
        carmove = Twist()
        carmove.linear.x = 0.1
        carmove.linear.y = -0.25
        for i in range(1, 30):
            self.cmdvel_pub.publish(carmove)
        time.sleep(1.0)
        carmove.linear.x = 0
        carmove.linear.y = 0
        self.cmdvel_pub.publish(carmove)

            
    def xArmGraspPlan(self, obj):
        """
        执行机械臂的抓取规划，包括移动到抓取姿态、关闭夹具、返回初始位置，并发布抓取完成状态
        
        参数:
            obj (abot_grasp_race.msg.ObjectInfo): 包含待抓取对象信息的对象
        """
        self.CameraiPointToGraspPose(obj)
        time.sleep(1)
        self.xArmPoseTxData(self.x, self.y, self.z, self.rotary)
        time.sleep(2)
        self.xArmGripperTxData(GRIPPER_CLOSE)
        time.sleep(2)
        self.xArmHome()
        time.sleep(2)
        self.grasp_lock = False
        self.grasp_goal_lock = False
        self.grasp_finish = True
        self.obj_cnt = 0
        self.grab_finished_pub.publish(True)
        rospy.loginfo("grasp object finished!!!!!!!!!!!!!!!!!!!!!!!!")

    def xArmFixGraspTest(self, obj):
        """
        在固定抓取模式下执行机械臂的抓取测试，包括打开夹具、移动到抓取姿态、关闭夹具、返回初始位置
        
        参数:
            obj (abot_grasp_race.msg.ObjectInfo): 包含待抓取对象信息的对象
        """
        self.xArmGripperTxData(GRIPPER_OPEN)
        time.sleep(1)
        self.CameraiPointToGraspPose(obj)
        time.sleep(1)
        self.xArmPoseTxData(self.x, self.y, self.z, self.rotary)
        time.sleep(2)
        self.xArmGripperTxData(GRIPPER_CLOSE)
        time.sleep(2)
        self.xArmHome(GRIPPER_OPEN)
        time.sleep(2)
        self.grasp_lock = False
        self.obj_cnt = 0
        rospy.loginfo("fix grasp test object finished!!!!!!!!!!!!!!!!!!!!!!!!")

    def xArmPlacePlan(self, obj):
        """
        执行机械臂的放置规划，包括移动到放置姿态、打开夹具、返回初始位置，并更新任务状态
        
        参数:
            obj (abot_grasp_race.msg.ObjectInfo): 包含待放置对象信息的对象
        """
        self.CameraiPointToGraspPose(obj)
        time.sleep(1)
        self.xArmPoseTxData(self.x, self.y, self.calib_data[5]+30, self.rotary)
        time.sleep(2)
        self.xArmGripperTxData(GRIPPER_OPEN)
        time.sleep(2)
        self.xArmHome(GRIPPER_OPEN)
        time.sleep(2)
        self.nav_step = 0
        self.task_cnt = self.task_cnt + 1
        self.grasp_finish = False
        self.place_goal_lock = False
        self.place_lock  = False
        self.obj_cnt = 0
        rospy.loginfo("place grasp object finished!!!!!!!!!!!!!!!!!!!!!!!!")

    def cbGetxArmPoseParam(self, config, level):
        """
        动态配置服务器的回调函数，用于更新机械臂的姿态参数
        
        参数:
            config (XarmPoseParamsConfig): 包含机械臂姿态参数的配置对象
            level (int): 配置级别
        
        返回值:
            config (XarmPoseParamsConfig): 更新后的配置对象
        """
        rospy.loginfo("Abot xArm Pose Calib")
        rospy.loginfo("x : %d", config.x)
        rospy.loginfo("y : %d", config.y)
        rospy.loginfo("z : %d", config.z)
        rospy.loginfo("rotary : %d", config.rotary)

        self.x = config.x
        self.y = config.y
        self.z = config.z
        self.rotary = config.rotary
        return config


    def grab_callback(self, msg):
        if msg.data is True:
            self.grab_cmd = True
            rospy.loginfo("Received grab command")

    def xArmPoseTxData(self, x, y, z, rotary=90):
        """
        向机械臂发送姿态控制数据
        
        参数:
            x (int): 机械臂X轴坐标
            y (int): 机械臂Y轴坐标
            z (int): 机械臂Z轴坐标
            rotary (int, optional): 机械臂旋转角度。默认为90
        """
        xArmPose = {"RikibotxArmCmd": 1,"Pose": {"x": x, "y": y, "z": z, "rotary": rotary},"time": self.speed}
        self.xArmCtrl.SerialWriteData(json.dumps(xArmPose))

    def xArmGripperTxData(self, gripper_step):
        """
        向机械臂发送夹具控制数据
        
        参数:
            gripper_step (int): 夹具的步进值（开合程度）
        """
        xArmGripper = {"RikibotxArmCmd": 2, "Gripper": gripper_step, "time": self.speed }
        self.xArmCtrl.SerialWriteData(json.dumps(xArmGripper))


    def xArmHome(self, gripper_step=GRIPPER_CLOSE):
        """
        控制机械臂回到初始位置，并设置夹具状态
        
        参数:
            gripper_step (int, optional): 夹具的步进值。默认为GRIPPER_CLOSE
        """
        self.xArmPoseTxData(0, 0, 250)
        time.sleep(1)
        self.xArmGripperTxData(gripper_step)

    def NavPoseConvert(self, pos, x=0, y=0):
        """
        将给定的位置和方向转换为ROS导航所需的Pose消息
        
        参数:
            pos (list): 包含X、Y坐标和偏航角的列表
            x (int, optional): X轴偏移量。默认为0
            y (int, optional): Y轴偏移量。默认为0
        
        返回值:
            geometry_msgs.msg.Pose: 转换后的ROS Pose消息
        """
        q = quaternion_from_euler(0, 0, np.deg2rad(pos[2]))

        return Pose(Point(pos[0]+x, pos[1]+y, 0), Quaternion(q[0], q[1], q[2], q[3]))

    def main(self):
        """
        ROS节点主循环，保持节点运行
        
        参数:
            无
        """
        rospy.loginfo("AbotxArmGrasp node initialized")
        rospy.spin()

    def shutdown(self):
        """
        ROS节点关闭时的回调函数，用于清理资源
        
        参数:
            无
        """
        
        rospy.loginfo("Shutting down abot_grasp_control node")
        self.xArmCtrl = XArmCtrl(self.port, self.baud)
        time.sleep(2)
        self.xArmCtrl.ServoStop()
        self.xArmCtrl.ServoStop()
        self.xArmHome()


if __name__ == '__main__':
    rospy.init_node('abot_grasp_control')
    node =AbotxArmGrasp()
    node.main()
