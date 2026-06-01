#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
顺序发送 /move_base_simple/goal，等价于在 RViz 逐一点 2D Nav Goal。
每点到位后自动发下一个，move_base 负责避障+规划。
用法: python2 waypoint_patrol.py [waypoints.yaml]
"""
import sys, os, math, time
import rospy, yaml
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray


def load_waypoints(path):
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)
    return cfg['waypoints']


class WaypointPatrol(object):
    def __init__(self, waypoints):
        rospy.init_node('waypoint_patrol')
        self.waypoints = waypoints
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self._status = None
        self._status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self._status_cb)
        rospy.loginfo('waypoint_patrol ready, %d waypoints', len(waypoints))

    def _status_cb(self, msg):
        self._status = msg

    def _wait_for_result(self, timeout=120.0):
        """等待当前 goal 完成 (status=3 SUCCEEDED)"""
        t0 = rospy.Time.now()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self._status and self._status.status_list:
                st = self._status.status_list[0].status
                if st == 3:  # SUCCEEDED
                    rospy.loginfo('  -> goal reached')
                    return True
                elif st == 4:  # ABORTED
                    rospy.logerr('  -> goal ABORTED')
                    return False
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr('  -> TIMEOUT')
                return False
            rate.sleep()
        return False

    def run(self):
        for i, wp in enumerate(self.waypoints):
            name = wp.get('name', 'wp%d' % i)
            rospy.loginfo('[%d/%d] %s -> (%.3f, %.3f, %.2f)',
                          i + 1, len(self.waypoints), name,
                          wp['x'], wp['y'], wp.get('yaw', 0.0))

            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = 'map'
            goal.pose.position.x = wp['x']
            goal.pose.position.y = wp['y']
            yaw = wp.get('yaw', 0.0)
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)

            # 等 0.5s 确保 move_base 消化上一个 goal 的状态
            rospy.sleep(0.5)
            self.goal_pub.publish(goal)
            rospy.loginfo('  goal sent, waiting...')

            if not self._wait_for_result():
                rospy.logerr('waypoint %s failed, abort patrol', name)
                return

        rospy.loginfo('all waypoints done')


if __name__ == '__main__':
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'waypoints.yaml')
    wps = load_waypoints(cfg_path)
    WaypointPatrol(wps).run()
