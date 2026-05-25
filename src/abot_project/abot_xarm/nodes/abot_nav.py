#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Authors: Rikibot kevin

import rospy
import os
import actionlib
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RikibotNav():
    def __init__(self):

	self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
                       'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

       
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Connected to move base server")

    def NavState(self):
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.logerr("ERROR:Timed out achieving goal")
            return 3
        else:
            state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
            rospy.sleep(1)
            return 1
        else:
            rospy.logerr("Goal failed with error code:"+str(self.goal_states[state]))
            return 2

    def SendGoal(self, goal):
        # Set up the next goal location
        self.goal = MoveBaseGoal()
        #self.goal.target_pose.pose = self.locations[locate]
        self.goal.target_pose.pose = goal
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal) #send goal to move_base


