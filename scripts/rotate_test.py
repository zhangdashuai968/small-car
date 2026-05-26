#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations
import math

rospy.init_node('rotate_test')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
print("Waiting for move_base action server...")
client.wait_for_server(timeout=rospy.Duration(10))
print("Connected!")

# Goal: rotate ~90 degrees in place (same position, different yaw)
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 0.0
goal.target_pose.pose.position.y = 0.0
goal.target_pose.pose.position.z = 0.0
# yaw = 90 degrees = pi/2
q = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
goal.target_pose.pose.orientation.x = q[0]
goal.target_pose.pose.orientation.y = q[1]
goal.target_pose.pose.orientation.z = q[2]
goal.target_pose.pose.orientation.w = q[3]

print("Sending rotation goal (yaw=90deg)...")
client.send_goal(goal)
print("Waiting for result (timeout 30s)...")
client.wait_for_result(timeout=rospy.Duration(30))
state = client.get_state()
print("Goal state:", state)
print("Done.")
