#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Send waypoints to move_base sequentially.

This runner is for navigate.launch + DWA tests. Use --nav-only for pass-only
navigation dry-runs that never call VLM or grasp.
"""

import argparse
import math
import os
import sys
import time

import actionlib
import rospy
import tf
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from riki_msgs.msg import Servo
from vl_locate.srv import GetObject

import tf.transformations as tft


SERVO_HOME = (88, 20)
GRASP_WAIT_SEC = 25
GOAL_STATE_NAMES = {
    GoalStatus.PENDING: 'PENDING',
    GoalStatus.ACTIVE: 'ACTIVE',
    GoalStatus.PREEMPTED: 'PREEMPTED',
    GoalStatus.SUCCEEDED: 'SUCCEEDED',
    GoalStatus.ABORTED: 'ABORTED',
    GoalStatus.REJECTED: 'REJECTED',
    GoalStatus.PREEMPTING: 'PREEMPTING',
    GoalStatus.RECALLING: 'RECALLING',
    GoalStatus.RECALLED: 'RECALLED',
    GoalStatus.LOST: 'LOST',
}


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def build_goal(wp):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = wp.get('frame', 'map')
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(wp['x'])
    goal.target_pose.pose.position.y = float(wp['y'])
    q = tft.quaternion_from_euler(0.0, 0.0, float(wp['yaw']))
    goal.target_pose.pose.orientation = Quaternion(*q)
    return goal


def load_waypoints(path):
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)
    return cfg['waypoints']


class MoveBaseWaypointRunner(object):
    def __init__(self, waypoints, nav_only, timeout,
                 yaw_align, yaw_tolerance, yaw_speed, yaw_timeout):
        rospy.init_node('move_base_waypoint_runner')
        self.waypoints = waypoints
        self.nav_only = nav_only
        self.timeout = timeout
        self.yaw_align = yaw_align
        self.yaw_tolerance = yaw_tolerance
        self.yaw_speed = yaw_speed
        self.yaw_timeout = yaw_timeout
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.servo_pub = rospy.Publisher('servo', Servo, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.vlm = None

        rospy.loginfo('waiting move_base action server ...')
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            raise RuntimeError('move_base action server unavailable')
        rospy.set_param('/grab', False)
        self._servo_home()

    def _servo_home(self):
        msg = Servo()
        msg.Servo1, msg.Servo2 = SERVO_HOME
        self.servo_pub.publish(msg)

    def _stop_base(self):
        self.cmd_pub.publish(Twist())

    def _lookup_yaw(self, frame):
        self.tf_listener.waitForTransform(
            frame, 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        _, quat = self.tf_listener.lookupTransform(
            frame, 'base_footprint', rospy.Time(0))
        return tft.euler_from_quaternion(quat)[2]

    def _align_yaw(self, wp):
        if not self.yaw_align:
            return True

        frame = wp.get('frame', 'map')
        target_yaw = float(wp['yaw'])
        deadline = rospy.Time.now() + rospy.Duration(self.yaw_timeout)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                current_yaw = self._lookup_yaw(frame)
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as exc:
                rospy.logwarn('yaw align waiting tf: %s', exc)
                rate.sleep()
                continue

            error = normalize_angle(target_yaw - current_yaw)
            if abs(error) <= self.yaw_tolerance:
                self._stop_base()
                rospy.loginfo('yaw aligned: target=%.3f current=%.3f err=%.3f',
                              target_yaw, current_yaw, error)
                return True

            cmd = Twist()
            angular = max(-self.yaw_speed, min(self.yaw_speed, 0.8 * error))
            if abs(angular) < 0.04:
                angular = 0.04 if error > 0.0 else -0.04
            cmd.angular.z = angular
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self._stop_base()
        rospy.logerr('yaw align TIMEOUT target=%.3f last_err=%.3f',
                     target_yaw, error if 'error' in locals() else float('nan'))
        return False

    def _do_action(self, action, obj):
        if self.nav_only or action in ('pass', 'finish'):
            if action == 'finish':
                rospy.set_param('/grab', False)
                self._servo_home()
            return True
        if action not in ('grab', 'place'):
            rospy.logerr('unknown action: %s', action)
            return False
        if self.vlm is None:
            rospy.loginfo('waiting /vlm_detection ...')
            rospy.wait_for_service('/vlm_detection', timeout=10.0)
            self.vlm = rospy.ServiceProxy('/vlm_detection', GetObject)
        is_grab = action == 'grab'
        rospy.set_param('/grab', is_grab)
        self._servo_home()
        rospy.sleep(1.0)
        self.vlm(obj)
        rospy.sleep(GRASP_WAIT_SEC)
        return True

    def run(self):
        for i, wp in enumerate(self.waypoints):
            action = 'pass' if self.nav_only else wp.get('action', 'pass')
            name = wp.get('name', 'wp%d' % i)
            rospy.loginfo('[%d/%d] %s -> (%.3f, %.3f, %.2f) action=%s',
                          i + 1, len(self.waypoints), name,
                          wp['x'], wp['y'], wp['yaw'], action)
            self.client.send_goal(build_goal(wp))
            finished = self.client.wait_for_result(rospy.Duration(self.timeout))
            if not finished:
                self.client.cancel_goal()
                rospy.logerr('goal %s TIMEOUT after %.1fs', name, self.timeout)
                return False
            state = self.client.get_state()
            if state != GoalStatus.SUCCEEDED:
                rospy.logerr('goal %s failed: state=%s',
                             name, GOAL_STATE_NAMES.get(state, str(state)))
                return False
            if not self._align_yaw(wp):
                rospy.logerr('goal %s yaw align failed', name)
                return False
            if not self._do_action(action, wp.get('object', '')):
                return False
        rospy.loginfo('all waypoints done')
        rospy.set_param('/grab', False)
        self._servo_home()
        return True


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('waypoints', nargs='?',
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                             'waypoints.yaml'))
    parser.add_argument('--nav-only', action='store_true')
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--skip-yaw-align', action='store_true')
    parser.add_argument('--yaw-tolerance', type=float, default=0.08)
    parser.add_argument('--yaw-speed', type=float, default=0.15)
    parser.add_argument('--yaw-timeout', type=float, default=20.0)
    return parser.parse_args(argv)


if __name__ == '__main__':
    args = parse_args(sys.argv[1:])
    try:
        ok = MoveBaseWaypointRunner(load_waypoints(args.waypoints),
                                    args.nav_only,
                                    args.timeout,
                                    not args.skip_yaw_align,
                                    args.yaw_tolerance,
                                    args.yaw_speed,
                                    args.yaw_timeout).run()
        sys.exit(0 if ok else 2)
    except (rospy.ROSException, RuntimeError) as exc:
        rospy.logerr('%s', exc)
        sys.exit(1)
