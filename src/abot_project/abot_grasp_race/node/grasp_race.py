#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Authors: Rikibot kevin

import rospy
import os, math, time, json
import numpy as np
import time
import cv2
from tf.transformations import quaternion_from_euler
from std_msgs.msg import UInt8, Float64
from dynamic_reconfigure.server import Server
from abot_xarm.cfg import XarmPoseParamsConfig
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from abot_grasp_race.msg import ObjectInfo, ObjectInfoArray
from xarm_driver import *
from nav import *
from geometry_msgs.msg import Twist


GRIPPER_CLOSE = 700
GRIPPER_OPEN = 150
GRIPPER_DEFAULT = 500

class AbotxArmGrasp():
    def __init__(self):
        self.x = rospy.get_param("~x", 0)
        self.y = rospy.get_param("~y", 0) 
        self.z = rospy.get_param("~z", 250)
        self.rotary = rospy.get_param("~rotary", 90)

        self.speed = rospy.get_param("~speed", 1000)  #Servo contorl speed ms
        self.port = rospy.get_param("~port", '/dev/ttyACM0')
        self.baud = rospy.get_param("~baud", 115200)

        self.fix_grasp_mode = rospy.get_param("~fix_grasp_mode", True)
        self.is_calibration_mode = rospy.get_param("~calibration_mode", False)
        self.calib_data = rospy.get_param("~calib_data", [55, -58, 513, 190, -65, -30])


        self.servo_run = rospy.get_param("~servo_run", False)
        self.servo_x = rospy.get_param("~servo_x", 10)
        self.grasp_order = rospy.get_param("~grasp_order", ["red", "green", "yellow"])
        self.place_offse_z = rospy.get_param("~place_offse_z", 30)
        self.nav_offset = rospy.get_param("~nav_offset", 0.4)
        self.ky = rospy.get_param("~ky", 10)
        self.ctrl_time = rospy.get_param("~ctrl_time", 150)
        self.obj_offset_x = rospy.get_param("~obj_offset_x", 30)
        self.obj_offset_y = rospy.get_param("~obj_offset_y", 50)

        self.grasp_pos = rospy.get_param("~grasp_pos")
        self.place_pos = rospy.get_param("~place_pos")
        self.end_pos = rospy.get_param("~end_pos")

        self.dir_path = os.path.dirname(os.path.realpath(__file__))

        self.dir_path = self.dir_path.replace('abot_grasp_race/node', 'abot_grasp_race/')
        self.dir_path += 'config/'

        if self.servo_run is True:
            os.system("rostopic pub -1 servo riki_msgs/Servo -- '90' '%d'" %(self.servo_x))

        rospy.Subscriber("/object_info", ObjectInfoArray, self.ObjectCallback)
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

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
        rospy.loginfo("grasp object finished!!!!!!!!!!!!!!!!!!!!!!!!")

    def xArmFixGraspTest(self, obj):
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


    def xArmPoseTxData(self, x, y, z, rotary=90):
        xArmPose = {"RikibotxArmCmd": 1,"Pose": {"x": x, "y": y, "z": z, "rotary": rotary},"time": self.speed}
        self.xArmCtrl.SerialWriteData(json.dumps(xArmPose))

    def xArmGripperTxData(self, gripper_step):
        xArmGripper = {"RikibotxArmCmd": 2, "Gripper": gripper_step, "time": self.speed }
        self.xArmCtrl.SerialWriteData(json.dumps(xArmGripper))


    def xArmHome(self, gripper_step=GRIPPER_CLOSE):
        self.xArmPoseTxData(0, 0, 250)
        time.sleep(1)
        self.xArmGripperTxData(gripper_step)

    def NavPoseConvert(self, pos, x=0, y=0):
        q = quaternion_from_euler(0, 0, np.deg2rad(pos[2]))

        return Pose(Point(pos[0]+x, pos[1]+y, 0), Quaternion(q[0], q[1], q[2], q[3]))

    def main(self):
        rospy.spin()

    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown xarm ...")


if __name__ == '__main__':
    rospy.init_node('abot_grasp_control')
    node =AbotxArmGrasp()
    node.main()
