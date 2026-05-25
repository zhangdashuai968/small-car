#!/usr/bin/env python3

import rospy
import os
import math
import time
import numpy as np
from vision_msgs.msg import BoundingBox2D, Detection2DArray,ObjectHypothesisWithPose,Detection2D
from kyle_robot_toolbox.robot_arm.arm4dof_uservo import Arm4DoFUServo


class xArmGrasp(object):
    def __init__(self):
        rospy.init_node("xArmsGrasp")
        r = rospy.Rate(5)
        self.PI = 3.1415

        self.cg_x = rospy.get_param("~cg_x", 25)
        self.cg_y = rospy.get_param("~cg_y", 120)
        self.cg_z = rospy.get_param("~cg_z", 210)
        self.obj_ok = rospy.get_param("~obj_ok", 10)

        self.place_pose = rospy.get_param("~place_pose", [120, 120, 80])

        self.obj_cnt = 0
        self.grasp_status = False
        self.object_pose = None

        dir_path = os.path.dirname(os.path.realpath(__file__))
        rospy.Subscriber("/object_point_info", Detection2D, self.ObjectCallback, queue_size=1)
        dir_path = dir_path.replace('robot_arm_grasp/node', 'robot_arm_grasp/')
        dir_path += 'config/'
        self.arm = Arm4DoFUServo(config_folder=dir_path, is_init_pose=True)
        self.arm.gripper_open(math.radians(30))


        while not rospy.is_shutdown():
            if self.grasp_status is True:
                rospy.loginfo("start grasp object")
                x, y, z = self.CameratoGraspPose()
                self.GraspObject(x, y, z)
                self.grasp_status = False
                self.obj_cnt = 0
            r.sleep()

    #获取目标坐标点
    def ObjectCallback(self, object_msg):
        self.object_info = object_msg
        self.object_pose = self.object_info.results[0].pose.pose.position
        if self.object_pose.z != 0:
            self.obj_cnt = self.obj_cnt + 1
        #print(self.object_info.results[0].pose.pose.position)
        if self.obj_cnt > self.obj_ok and self.grasp_status is False:
            self.grasp_status = True

    #抓取流程
    def GraspObject(self, x, y, z):
        gx, gy, gz, gpitch = self.arm.get_tool_pose()
        self.arm.gripper_open(math.radians(30))
        time.sleep(1)
        self.arm.set_tool_pose([x, y, z])
        time.sleep(1)
        self.arm.gripper_close()
        time.sleep(1)
        gx, gy, gz, gpitch = self.arm.get_tool_pose()
        rospy.loginfo("move grasp current gripper pose: %f, %f, %f, %f", \
                gx, gy, gz, np.degrees(gpitch))
        self.arm.set_tool_pose(self.place_pose)
        time.sleep(1)
        self.arm.gripper_open(math.radians(30))
        time.sleep(1)
        self.arm.home()
        time.sleep(2)

    #摄像头与机械臂的坐标转换
    def CameratoGraspPose(self):
        rospy.loginfo("current object xyz: %d, %d, %d", \
                self.object_pose.x, self.object_pose.y, self.object_pose.z)
        gx, gy, gz, gpitch = self.arm.get_tool_pose()
        rospy.loginfo("current gripper pose: %f, %f, %f, %f", \
                gx, gy, gz, np.degrees(gpitch))
        pitch_angle = self.PI +  gpitch
        dz = math.sqrt(self.object_pose.y*self.object_pose.y+self.object_pose.z*self.object_pose.z)
        theat1 = (math.atan2(abs(self.object_pose.y), self.object_pose.z))
        if self.object_pose.y >= 0 :
            theat = pitch_angle - theat1
        else:
            theat = pitch_angle + theat1

        grasp_px = gx + dz*math.cos(theat) - self.cg_y
        grasp_pz = dz*math.sin(theat) + self.cg_z
        if self.object_pose.x > 0 :
            grasp_py = -self.object_pose.x
        else:
            grasp_py = self.cg_x - self.object_pose.x 
        rospy.loginfo("grasp pose gx : %f , gy: %f, gz: %f", grasp_px, grasp_py,  grasp_pz)
        return grasp_px, grasp_py, grasp_pz



    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown object_xyz_node ...")



if __name__ == '__main__':
    try:
        driver = xArmGrasp()
        rospy.spin()
    except Exception as e:
        print("Init xArm Grasp Failed...")
        raise e
