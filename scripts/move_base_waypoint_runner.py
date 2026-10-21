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
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from riki_msgs.msg import Servo
from std_srvs.srv import Empty
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
        self.initialpose_pub = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=1)
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
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.03)

    def _call_empty_service(self, service_name, timeout=1.0):
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
            rospy.ServiceProxy(service_name, Empty)()
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn('%s call skipped: %s', service_name, exc)
            return False

    def _request_nomotion_updates(self, count):
        for _ in range(max(0, int(count))):
            self._call_empty_service('/request_nomotion_update', timeout=0.8)
            rospy.sleep(0.15)

    def _lookup_yaw(self, frame):
        self.tf_listener.waitForTransform(
            frame, 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        _, quat = self.tf_listener.lookupTransform(
            frame, 'base_footprint', rospy.Time(0))
        return tft.euler_from_quaternion(quat)[2]

    def _lookup_pose(self, frame):
        self.tf_listener.waitForTransform(
            frame, 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        trans, quat = self.tf_listener.lookupTransform(
            frame, 'base_footprint', rospy.Time(0))
        return trans[0], trans[1], tft.euler_from_quaternion(quat)[2]

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

    def _rotate_to_yaw(self, frame, target_yaw, speed, timeout, label):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                current_yaw = self._lookup_yaw(frame)
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as exc:
                rospy.logwarn('%s waiting tf: %s', label, exc)
                rate.sleep()
                continue

            error = normalize_angle(target_yaw - current_yaw)
            if abs(error) <= self.yaw_tolerance:
                self._stop_base()
                rospy.loginfo('%s reached: target=%.3f current=%.3f err=%.3f',
                              label, target_yaw, current_yaw, error)
                return True

            cmd = Twist()
            angular = max(-speed, min(speed, 0.8 * error))
            if abs(angular) < 0.04:
                angular = 0.04 if error > 0.0 else -0.04
            cmd.angular.z = angular
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self._stop_base()
        rospy.logerr('%s TIMEOUT target=%.3f last_err=%.3f',
                     label, target_yaw, error if 'error' in locals() else float('nan'))
        return False

    def _publish_initialpose(self, frame, x, y, yaw, cov_xy, cov_yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        quat = tft.quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation = Quaternion(*quat)
        msg.pose.covariance[0] = cov_xy
        msg.pose.covariance[7] = cov_xy
        msg.pose.covariance[35] = cov_yaw

        for _ in range(3):
            msg.header.stamp = rospy.Time.now()
            self.initialpose_pub.publish(msg)
            rospy.sleep(0.12)

    def _localize_checkpoint(self, wp):
        if not wp.get('localize_checkpoint', False):
            return True

        frame = wp.get('frame', 'map')
        target_x = float(wp['x'])
        target_y = float(wp['y'])
        target_yaw = float(wp['yaw'])
        max_distance = float(wp.get('checkpoint_max_distance', 0.45))
        max_yaw_error = float(wp.get('checkpoint_max_yaw_error', 0.61))
        cov_xy = float(wp.get('checkpoint_cov_xy', 0.05))
        cov_yaw = float(wp.get('checkpoint_cov_yaw', 0.12))
        sweep_yaw = abs(float(wp.get('checkpoint_sweep_yaw', 0.21)))
        sweep_speed = abs(float(wp.get('checkpoint_sweep_speed', 0.12)))
        sweep_timeout = float(wp.get('checkpoint_sweep_timeout', 8.0))
        update_count = int(wp.get('checkpoint_updates', 3))
        pause = float(wp.get('checkpoint_pause', 0.5))

        try:
            current_x, current_y, current_yaw = self._lookup_pose(frame)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as exc:
            rospy.logerr('localize checkpoint failed to read pose: %s', exc)
            return False

        distance = math.hypot(target_x - current_x, target_y - current_y)
        yaw_error = normalize_angle(target_yaw - current_yaw)
        if distance > max_distance or abs(yaw_error) > max_yaw_error:
            rospy.logerr(
                'localize checkpoint guard failed: dist=%.3f yaw_err=%.3f',
                distance, yaw_error)
            return False

        rospy.loginfo('localize checkpoint start at (%.3f, %.3f, %.3f)',
                      target_x, target_y, target_yaw)
        self._stop_base()
        rospy.sleep(pause)
        self._publish_initialpose(frame, target_x, target_y, target_yaw,
                                  cov_xy, cov_yaw)
        rospy.sleep(pause)
        self._request_nomotion_updates(update_count)

        targets = [
            ('checkpoint sweep left', normalize_angle(target_yaw + sweep_yaw)),
            ('checkpoint sweep right', normalize_angle(target_yaw - sweep_yaw)),
            ('checkpoint sweep return', target_yaw),
        ]
        for label, yaw in targets:
            if not self._rotate_to_yaw(frame, yaw, sweep_speed,
                                       sweep_timeout, label):
                return False
            self._request_nomotion_updates(1)

        self._call_empty_service('/move_base/clear_costmaps', timeout=1.0)
        rospy.sleep(pause)
        rospy.loginfo('localize checkpoint done')
        return True

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
            if not self._localize_checkpoint(wp):
                rospy.logerr('goal %s localize checkpoint failed', name)
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
