#!/usr/bin/env python3

import rospy
import os
import actionlib
import time
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from std_msgs.msg import Int16, Int8, Float64
from std_msgs.msg import Header
import math
import numpy as np
from kyle_robot_toolbox.robot_arm.arm4dof_uservo import Arm4DoFUServo
from kyle_robot_toolbox.system import sleep_s, get_cur_time_s


class RikibotArmDriver(object):
    def __init__(self):
        rospy.init_node("Robot_Arms_Driver")

        r = rospy.Rate(10)
        self.PI = 3.1415926
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.last_position = None
        self.last_gripper = None
        self.cnt  = 1

        dir_path = dir_path.replace('robot_arm_driver/node', 'robot_arm_driver/')
        dir_path += 'config/'
        self.arm = Arm4DoFUServo(config_folder=dir_path, is_init_pose=False)

        rospy.Subscriber('joint_states', JointState, self.joint_callback)
        rospy.loginfo("start driver 4dof arm")

        while not rospy.is_shutdown():
            r.sleep()

    def joint_callback(self, joint_msg):
        joint_position = list(joint_msg.position)[:-2]
        gripper_position = list(joint_msg.position)[-1]
        np_joint = np.array(joint_position)
        joint_position = np.round(np_joint, 4)
        joint_position[0] = -joint_position[0]
        joint_position[1] = joint_position[1] - math.pi/2
        value = (np.array(self.last_position) == np.array(joint_position)).all()
        if bool(value) is not True :
            self.arm.set_joint_angle_list(joint_position, is_wait=False)
            time.sleep(0.1)
        if gripper_position != self.last_gripper:
            self.arm.set_gripper_angle(abs(gripper_position), T=0.1, max_power=1000) 
            self.last_gripper = gripper_position
        self.last_position = joint_position

if __name__ == '__main__':
    try:
        driver = RikibotArmDriver()
        rospy.spin()

    except Exception as e:
        rospy.loginfo("Init Rikibot Arm Failed...")
        raise e
