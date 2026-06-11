#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from riki_msgs.msg import Servo
from vl_locate.srv import GetObject
import tf.transformations as tf


class AutoNavigationGrasp:
    def __init__(self):
        rospy.init_node('auto_navigation_grasp')

        # Move base client
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()

        # Publishers
        self.servo_pub = rospy.Publisher('servo', Servo, queue_size=1)

        # Service client
        rospy.wait_for_service('/vlm_detection')
        self.vlm_client = rospy.ServiceProxy('/vlm_detection', GetObject)

        # 10点任务: 1路过 2抓 3放 4路过 5抓 6放 7路过 8抓 9放 10终点
        self.tasks = [
            {'name': 'point_01_pass', 'pose': (-0.571, -1.560, 0.121), 'action': 'pass', 'object': None},
            {'name': 'point_02_grab', 'pose': (-0.417, -2.791, 0.068), 'action': 'grab', 'object': '苹果'},
            {'name': 'point_03_place', 'pose': (-0.571, -1.560, 0.121), 'action': 'place', 'object': '图片中心点'},
            {'name': 'point_04_pass', 'pose': (-1.810, -1.618, 0.106), 'action': 'pass', 'object': None},
            {'name': 'point_05_grab', 'pose': (-1.544, -2.873, 0.351), 'action': 'grab', 'object': '榴莲'},
            {'name': 'point_06_place', 'pose': (-1.810, -1.618, 0.106), 'action': 'place', 'object': '图片中心点'},
            {'name': 'point_07_pass', 'pose': (-3.130, -2.130, 0.633), 'action': 'pass', 'object': None},
            {'name': 'point_08_grab', 'pose': (-2.298, -3.067, 0.851), 'action': 'grab', 'object': '西瓜'},
            {'name': 'point_09_place', 'pose': (-3.130, -2.130, 0.633), 'action': 'place', 'object': '图片中心点'},
            {'name': 'point_10_finish', 'pose': (-4.653, -1.317, 0.876), 'action': 'finish', 'object': None},
        ]

    def send_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        q = tf.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)

        self.move_base.send_goal(goal)
        return self.move_base.wait_for_result()

    def set_servo(self):
        servo_msg = Servo()
        servo_msg.Servo1 = 88
        servo_msg.Servo2 = 20
        self.servo_pub.publish(servo_msg)

    def set_grab_param(self, grab):
        rospy.set_param('/grab', grab)

    def call_vlm_detection(self, object_name):
        try:
            response = self.vlm_client(object_name)
            return response
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def run_task_action(self, task):
        action = task['action']
        if action == 'pass':
            rospy.loginfo("Pass point: %s", task['name'])
            return
        if action == 'finish':
            self.set_grab_param(0)
            self.set_servo()
            rospy.loginfo("Final point reached - task complete")
            return

        grab = action == 'grab'
        self.set_grab_param(1 if grab else 0)
        rospy.loginfo("%s: %s", "Grabbing" if grab else "Placing", task['object'])
        self.call_vlm_detection(task['object'])
        time.sleep(5)

    def run(self):
        rospy.loginfo("Starting 10-point auto navigation and grasp sequence")

        self.set_servo()
        self.set_grab_param(0)
        time.sleep(5)

        for i, task in enumerate(self.tasks):
            x, y, yaw = task['pose']

            rospy.loginfo("Navigating to point %d/%d %s: (%.2f, %.2f, %.2f)",
                          i + 1, len(self.tasks), task['name'], x, y, yaw)

            if self.send_goal(x, y, yaw):
                rospy.loginfo("Reached point %d: %s", i + 1, task['name'])
                self.run_task_action(task)
            else:
                rospy.logwarn("Failed to reach point %d: %s", i + 1, task['name'])

        rospy.loginfo("10-point auto navigation and grasp sequence completed")


if __name__ == '__main__':
    try:
        auto_nav = AutoNavigationGrasp()
        auto_nav.run()
    except rospy.ROSInterruptException:
        pass
