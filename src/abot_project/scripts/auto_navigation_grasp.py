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
        
        # Navigation points (x, y, yaw in radians)
        self.waypoints = [
            (3.6, -0.9, 1.6),   # Point 1
            (2.4, -0.9, 1.6),   # Point 2  
            (3.6, -2.13, 1.6),  # Point 3
            (2.4, -2.13, 1.6),  # Point 4
            (3.6, -3.33, 1.6),  # Point 5
            (2.4, -3.33, 1.6),  # Point 6
            (2.4, -2.0, 0.0),   # Point 7
        ]
        
        # Actions for each point (True=grab, False=place)
        self.actions = [True, False, True, False, True, False, None]
        
        # Object names for each point
        self.objects = ['蓝方块', '图片中心点', '红方块', '图片中心点', '绿方块', '图片中心点', None]

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

    def run(self):
        rospy.loginfo("Starting auto navigation and grasp sequence")
        
        # Set initial servo position
        self.set_servo()
        time.sleep(2)
        
        for i, (waypoint, action) in enumerate(zip(self.waypoints, self.actions)):
            x, y, yaw = waypoint
            
            rospy.loginfo("Navigating to point %d: (%.2f, %.2f, %.2f)", i+1, x, y, yaw)
            
            # Navigate to waypoint
            if self.send_goal(x, y, yaw):
                rospy.loginfo("Reached point %d", i+1)
                
                if action is not None:
                    # Set grab parameter
                    self.set_grab_param(1 if action else 0)
                    
                    # Call VLM detection
                    object_name = self.objects[i]
                    rospy.loginfo("Detecting: %s", object_name)
                    self.call_vlm_detection(object_name)
                    
                    time.sleep(3)  # Wait for grasp/place completion
                else:
                    rospy.loginfo("Final point reached - task complete")
            else:
                rospy.logwarn("Failed to reach point %d", i+1)
        
        rospy.loginfo("Auto navigation and grasp sequence completed")

if __name__ == '__main__':
    try:
        auto_nav = AutoNavigationGrasp()
        auto_nav.run()
    except rospy.ROSInterruptException:
        pass