#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
硬编码相对位移比赛脚本：走固定米数 -> 执行动作 -> 等待 -> 继续下一段。

本脚本不发 move_base goal，不跑 DWA/TEB；只按固定距离向 /cmd_vel 发
linear.x / linear.y，并在每段后刹停。距离来自赛前给定绝对点位差分，
默认假设车头朝赛道 +x，车体 y 正方向为向左。

前置：
  roslaunch abot bringup.launch
  roslaunch abot localize.launch        # 可选，用于观察定位；本脚本不依赖 move_base
  roslaunch ZachLab_grasp grasp.launch
  roslaunch vl_locate vl_locate.launch

用法：
  python2 hardcoded_race.py
  python2 hardcoded_race.py --nav-only
  python2 hardcoded_race.py --dry-run
"""

from __future__ import print_function

import argparse
import math
import signal
import sys
import time


# 按用户给定的绝对点位差分：
#   (0,0)
#   -> (-0.6,-1.8) -> (-0.6,-3.0) 抓取 -> (-0.6,-1.8) 放置
#   -> (-0.6,-0.6) -> (-1.8,-0.6) -> (-1.8,-1.8) -> (-1.8,-3.0) 抓取
#   -> (-1.8,-1.8) 放置 -> (-1.8,-0.6) -> (-3.0,-0.6) -> (-3.0,-1.8)
#   -> (-3.0,-3.0) 抓取 -> (-3.0,-1.8) 放置 -> (-3.4,-3.4) 终点
# action: pass / grab / place / finish
# moves: 车体系相对位移，x 正=前进，y 正=左移，单位 m。
ROUTE = [
    {
        'name': 'to_col1_place_lane',
        'moves': [('x', -0.600), ('y', -1.800)],
        'action': 'pass',
        'object': '',
    },
    {
        'name': 'col1_grab',
        'moves': [('y', -1.200)],
        'action': 'grab',
        'object': '西瓜',
    },
    {
        'name': 'col1_place',
        'moves': [('y', 1.200)],
        'action': 'place',
        'object': '图片中心点',
    },
    {
        'name': 'col1_back_lane',
        'moves': [('y', 1.200)],
        'action': 'pass',
        'object': '',
    },
    {
        'name': 'to_col2_entry',
        'moves': [('x', -1.200)],
        'action': 'pass',
        'object': '',
    },
    {
        'name': 'to_col2_place_lane',
        'moves': [('y', -1.200)],
        'action': 'pass',
        'object': '',
    },
    {
        'name': 'col2_grab',
        'moves': [('y', -1.200)],
        'action': 'grab',
        'object': '火龙果',
    },
    {
        'name': 'col2_place',
        'moves': [('y', 1.200)],
        'action': 'place',
        'object': '图片中心点',
    },
    {
        'name': 'col2_back_lane',
        'moves': [('y', 1.200)],
        'action': 'pass',
        'object': '',
    },
    {
        'name': 'to_col3_entry',
        'moves': [('x', -1.200)],
        'action': 'pass',
        'object': '',
    },
    {
        'name': 'to_col3_place_lane',
        'moves': [('y', -1.200)],
        'action': 'pass',
        'object': '',
    },
    {
        'name': 'col3_grab',
        'moves': [('y', -1.200)],
        'action': 'grab',
        'object': '梨',
    },
    {
        'name': 'col3_place',
        'moves': [('y', 1.200)],
        'action': 'place',
        'object': '图片中心点',
    },
    {
        'name': 'finish',
        'moves': [('x', -0.400), ('y', -1.600)],
        'action': 'finish',
        'object': '',
    },
]


SERVO_HOME = (88, 20)
RATE_HZ = 20
DEFAULT_LIN_SPEED = 0.16
DEFAULT_TURN_SPEED = 0.30
DEFAULT_SETTLE_WAIT = 0.8
DEFAULT_ACTION_WAIT = 25.0
DEFAULT_MAX_STEP_SECONDS = 24.0
MAX_STEP_METERS = 2.0
MAX_TURN_RADIANS = math.pi

rospy = None
Twist = None
Servo = None
GetObject = None


def load_ros():
    global rospy, Twist, Servo, GetObject
    if rospy is not None:
        return
    import rospy as _rospy
    from geometry_msgs.msg import Twist as _Twist
    from riki_msgs.msg import Servo as _Servo
    from vl_locate.srv import GetObject as _GetObject
    rospy = _rospy
    Twist = _Twist
    Servo = _Servo
    GetObject = _GetObject


def fmt_moves(moves):
    return ', '.join('%s%+.3fm' % (axis, meters) for axis, meters in moves)


def summarize_route(route, args):
    x = 0.0
    y = 0.0
    total_motion = 0.0
    print('hardcoded_race dry-run')
    print('speed=%.3fm/s x_scale=%.3f y_scale=%.3f settle=%.1fs action_wait=%.1fs' %
          (args.speed, args.x_scale, args.y_scale, args.settle_wait, args.action_wait))
    for idx, task in enumerate(route, 1):
        if idx < args.from_step:
            continue
        task_motion = 0.0
        for axis, meters in task['moves']:
            if axis == 'x':
                x += meters
                scaled = abs(meters * args.x_scale)
            elif axis == 'y':
                y += meters
                scaled = abs(meters * args.y_scale)
            else:
                scaled = 0.0
            task_motion += scaled / args.speed
        total_motion += task_motion
        print('%02d %-24s moves=[%s] action=%s object=%s -> est=(%.3f, %.3f) move %.1fs' %
              (idx, task['name'], fmt_moves(task['moves']), task['action'],
               task.get('object', ''), x, y, task_motion))
    print('estimated motion time: %.1fs, plus waits/actions' % total_motion)


class HardcodedRace(object):
    def __init__(self, args):
        load_ros()
        rospy.init_node('hardcoded_race')
        self.args = args
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.servo_pub = rospy.Publisher('servo', Servo, queue_size=1, latch=True)
        self.vlm = None

        rospy.on_shutdown(self.safe_shutdown)
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

        rospy.set_param('/grab', False)
        rospy.set_param('/success', False)
        rospy.sleep(0.5)
        self._servo_home()
        self._stop()

    def _on_signal(self, *_):
        rospy.logwarn('interrupted, stop + servo home')
        self.safe_shutdown()
        rospy.signal_shutdown('interrupted')
        sys.exit(0)

    def _servo_home(self):
        msg = Servo()
        msg.Servo1, msg.Servo2 = SERVO_HOME
        self.servo_pub.publish(msg)

    def _stop(self):
        zero = Twist()
        for _ in range(4):
            self.cmd_pub.publish(zero)
            rospy.sleep(0.03)

    def safe_shutdown(self):
        try:
            self._stop()
        except Exception:
            pass
        try:
            rospy.set_param('/grab', False)
            self._servo_home()
        except Exception:
            pass

    def _wait(self, seconds, label):
        if self.args.skip_waits or seconds <= 0.0:
            return True
        rospy.loginfo('%s wait %.1fs', label, seconds)
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            rate.sleep()
        return not rospy.is_shutdown()

    def _drive_axis(self, axis, meters):
        if abs(meters) < 0.001:
            return True
        if axis not in ('x', 'y'):
            rospy.logerr('unknown move axis: %s', axis)
            return False
        if abs(meters) > MAX_STEP_METERS:
            rospy.logerr('step too long: %s %.3fm > %.3fm', axis, meters, MAX_STEP_METERS)
            return False

        scale = self.args.x_scale if axis == 'x' else self.args.y_scale
        duration = abs(meters * scale) / self.args.speed
        if duration > self.args.max_step_seconds:
            rospy.logerr('step duration too long: %s %.3fm -> %.1fs', axis, meters, duration)
            return False

        cmd = Twist()
        sign = 1.0 if meters > 0.0 else -1.0
        if axis == 'x':
            cmd.linear.x = sign * self.args.speed
        else:
            cmd.linear.y = sign * self.args.speed

        rospy.loginfo('move %s%+.3fm speed=%.3f duration=%.2fs',
                      axis, meters, self.args.speed, duration)
        deadline = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self._stop()
        return self._wait(self.args.settle_wait, 'settle')

    def _turn(self, radians):
        if abs(radians) < 0.005:
            return True
        if abs(radians) > MAX_TURN_RADIANS:
            rospy.logerr('turn too large: %.3frad', radians)
            return False

        duration = abs(radians) / self.args.turn_speed
        if duration > self.args.max_step_seconds:
            rospy.logerr('turn duration too long: %.3frad -> %.1fs', radians, duration)
            return False

        cmd = Twist()
        cmd.angular.z = (1.0 if radians > 0.0 else -1.0) * self.args.turn_speed
        rospy.loginfo('turn %+.3frad speed=%.3f duration=%.2fs',
                      radians, self.args.turn_speed, duration)
        deadline = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self._stop()
        return self._wait(self.args.settle_wait, 'settle')

    def _do_action(self, action, obj):
        if self.args.nav_only and action in ('grab', 'place'):
            rospy.logwarn('nav-only: skip %s %s', action, obj)
            return self._wait(self.args.settle_wait, 'nav-only action gap')
        if action == 'pass':
            return True
        if action == 'finish':
            rospy.set_param('/grab', False)
            rospy.set_param('/success', False)
            self._servo_home()
            return self._wait(self.args.settle_wait, 'finish')
        if action not in ('grab', 'place'):
            rospy.logerr('unknown action: %s', action)
            return False

        try:
            if self.vlm is None:
                rospy.loginfo('waiting /vlm_detection ...')
                rospy.wait_for_service('/vlm_detection', timeout=10.0)
                self.vlm = rospy.ServiceProxy('/vlm_detection', GetObject)
            rospy.set_param('/grab', action == 'grab')
            rospy.set_param('/success', False)
            self._servo_home()
            rospy.sleep(0.8)
            rospy.loginfo('%s object=%s', action, obj)
            response = self.vlm(obj)
            if hasattr(response, 'success') and not response.success:
                rospy.logerr('vlm_detection failed: %s',
                             getattr(response, 'message', 'no message'))
                return False
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr('%s action failed: %s', action, exc)
            rospy.set_param('/grab', False)
            self._servo_home()
            return False
        return self._wait(self.args.action_wait, action)

    def run(self):
        rospy.loginfo('hardcoded_race: %d tasks, from_step=%d',
                      len(ROUTE), self.args.from_step)
        for idx, task in enumerate(ROUTE, 1):
            if idx < self.args.from_step:
                continue
            rospy.loginfo('[%d/%d] %s moves=[%s] action=%s',
                          idx, len(ROUTE), task['name'],
                          fmt_moves(task['moves']), task['action'])
            for move in task['moves']:
                axis, value = move
                if axis == 'yaw':
                    ok = self._turn(value)
                else:
                    ok = self._drive_axis(axis, value)
                if not ok:
                    rospy.logerr('motion failed at task %s', task['name'])
                    return False
            if not self._do_action(task['action'], task.get('object', '')):
                rospy.logerr('action failed at task %s', task['name'])
                return False
        rospy.loginfo('hardcoded_race done')
        self.safe_shutdown()
        return True


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dry-run', action='store_true',
                        help='print the fixed sequence without importing ROS')
    parser.add_argument('--nav-only', action='store_true',
                        help='skip grab/place actions')
    parser.add_argument('--from-step', type=int, default=1,
                        help='1-based task index to resume from')
    parser.add_argument('--speed', type=float, default=DEFAULT_LIN_SPEED,
                        help='linear speed in m/s')
    parser.add_argument('--turn-speed', type=float, default=DEFAULT_TURN_SPEED,
                        help='turn speed in rad/s')
    parser.add_argument('--x-scale', type=float, default=1.0,
                        help='duration multiplier for x moves')
    parser.add_argument('--y-scale', type=float, default=1.0,
                        help='duration multiplier for y moves')
    parser.add_argument('--settle-wait', type=float, default=DEFAULT_SETTLE_WAIT,
                        help='stop-and-settle wait after each motion')
    parser.add_argument('--action-wait', type=float, default=DEFAULT_ACTION_WAIT,
                        help='fixed wait after grab/place service call')
    parser.add_argument('--max-step-seconds', type=float,
                        default=DEFAULT_MAX_STEP_SECONDS)
    parser.add_argument('--skip-waits', action='store_true')
    args = parser.parse_args(argv)
    if args.from_step < 1 or args.from_step > len(ROUTE):
        parser.error('--from-step must be in [1, %d]' % len(ROUTE))
    if args.speed <= 0.0:
        parser.error('--speed must be positive')
    if args.turn_speed <= 0.0:
        parser.error('--turn-speed must be positive')
    if args.x_scale <= 0.0 or args.y_scale <= 0.0:
        parser.error('--x-scale and --y-scale must be positive')
    return args


if __name__ == '__main__':
    parsed = parse_args(sys.argv[1:])
    if parsed.dry_run:
        summarize_route(ROUTE, parsed)
        sys.exit(0)
    runner = HardcodedRace(parsed)
    sys.exit(0 if runner.run() else 2)
