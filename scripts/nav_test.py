#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations
import math

rospy.init_node('nav_test')

# Get current pose from AMCL
current_pose = [None]
def amcl_cb(msg):
    current_pose[0] = msg.pose.pose

sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_cb)
rospy.sleep(2)
sub.unregister()

if current_pose[0] is None:
    print("ERROR: No AMCL pose received!")
    exit(1)

p = current_pose[0].position
o = current_pose[0].orientation
# Get current yaw
euler = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
yaw = euler[2]
print("Current pose: x=%.3f, y=%.3f, yaw=%.1f deg" % (p.x, p.y, math.degrees(yaw)))

# Goal: 1m ahead in current facing direction
dist = 1.0
goal_x = p.x + dist * math.cos(yaw)
goal_y = p.y + dist * math.sin(yaw)
print("Goal: x=%.3f, y=%.3f (same yaw)" % (goal_x, goal_y))

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
print("Waiting for move_base...")
client.wait_for_server(timeout=rospy.Duration(10))

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = goal_x
goal.target_pose.pose.position.y = goal_y
goal.target_pose.pose.position.z = 0.0
goal.target_pose.pose.orientation = o  # keep same yaw

print("Sending goal...")
client.send_goal(goal)

# Monitor progress
rate = rospy.Rate(2)
while not rospy.is_shutdown():
    state = client.get_state()
    if state in [2, 3, 4, 5]:  # terminal states
        break
    print("State: %d, time elapsed: %.1fs" % (state, (rospy.Time.now() - goal.target_pose.header.stamp).to_sec()))
    rate.sleep()

state_names = {0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED',
               4: 'ABORTED', 5: 'REJECTED', 6: 'PREEMPTED', 8: 'RECALLED'}
print("Final state: %d (%s)" % (state, state_names.get(state, 'UNKNOWN')))
print("Done.")
