#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Authors: Rikibot kevin

"""
Navigation module for controlling robot movement using ROS.

This module provides a class for sending navigation goals to a robot
using the move_base action server.
"""

import rospy
import os
import actionlib
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Nav():
    """
    A class for controlling robot navigation using the move_base action server.
    
    This class provides methods for sending navigation goals to the robot and
    checking the status of navigation goals.
    """
    def __init__(self):
        """
        Initializes the Nav class.
        
        This method sets up the move_base action client and waits for the server to be available.
        It also defines the possible goal states for navigation.
        """

	self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
                       'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

       
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Connected to move base server")

    def NavState(self):
        """
        Checks the status of the current navigation goal.
        
        This method waits for the result of the navigation goal and returns a status code:
        - 1: Goal succeeded
        - 2: Goal failed
        - 3: Timed out achieving goal
        
        Returns:
            int: Status code indicating the result of the navigation goal.
        """
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
        """
        Sends a navigation goal to the move_base action server.
        
        Args:
            goal (Pose): The goal pose for navigation.
        """
        # Set up the next goal location
        self.goal = MoveBaseGoal()
        #self.goal.target_pose.pose = self.locations[locate]
        self.goal.target_pose.pose = goal
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal) #send goal to move_base


