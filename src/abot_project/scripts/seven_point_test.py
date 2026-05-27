#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
七点一次性可达性测试（绕开 move_base）。

硬编码 7 个 map 系点，逐点 L 形分解直达 + 原地转到目标朝向，全程锁航向压漂、
只沿 xy 轴走（满足比赛"碰挡板即判负"硬约束）。位姿来源 tf map->base_footprint，
由 amcl(localize.launch) + EKF(bringup) 提供。不抓取、不放置，仅验证 7 点都能到。

前置(手动启动)：
  roslaunch abot bringup.launch
  roslaunch abot localize.launch      # amcl 读 comp.yaml, 提供 map->base_footprint

用法：python2 seven_point_test.py      (或 rosrun abot seven_point_test.py)
中断(Ctrl-C)：立即停车。
"""

import sys
import math
import signal

import rospy
import tf
from geometry_msgs.msg import Twist


# 7 个测试点 (map 系 x, y, yaw[rad])
WAYPOINTS = [
    (3.6, 0.6, 0.0),   # Point 1
    (2.4, 0.6, 0.0),   # Point 2
    (3.6, 1.8, 0.0),   # Point 3
    (2.4, 1.8, 0.0),   # Point 4
    (3.6, 2.6, 0.0),   # Point 5
    (2.4, 2.6, 0.0),   # Point 6
    (2.4, 2.0, 0.0),   # Point 7
]

# --- 原语控制器参数 (与 auto_task_runner 一致) ---
V_LIN      = 0.20   # m/s   直线/横移线速度上限
V_ROT      = 0.40   # rad/s 原地转角速度上限
V_LIN_MIN  = 0.04   # m/s   最小线速度(克服静摩擦)
V_ROT_MIN  = 0.10   # rad/s 最小转角速度
K_LIN      = 4.0    # 线速度 P 增益
K_LIN_HOLD = 5.0    # 非主运动轴 P 增益(压制漂移)
K_YAW      = 1.5    # 角速度 P 增益
POS_TOL    = 0.04   # m     终点位置容差
YAW_TOL    = 0.03   # rad   终点角度容差(~1.7deg)
RATE_HZ    = 20
STEP_TIMEOUT = 30.0  # s    单步超时(fail-fast)

BASE_FRAME = 'base_footprint'
MAP_FRAME  = 'map'


def _norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SevenPointTest(object):
    def __init__(self):
        rospy.init_node('seven_point_test')
        self._tf = tf.TransformListener()
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        rospy.loginfo('waiting tf %s->%s ...', MAP_FRAME, BASE_FRAME)
        try:
            self._tf.waitForTransform(MAP_FRAME, BASE_FRAME, rospy.Time(0), rospy.Duration(10.0))
        except tf.Exception:
            rospy.logwarn('no %s->%s tf, 确认 localize.launch 已启动且 amcl 已收敛', MAP_FRAME, BASE_FRAME)

        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigint)
        rospy.sleep(0.5)

    def _cur(self):
        try:
            trans, rot = self._tf.lookupTransform(MAP_FRAME, BASE_FRAME, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return (trans[0], trans[1], yaw)

    def _stop(self):
        t = Twist()
        for _ in range(3):
            self._pub.publish(t)
            rospy.sleep(0.02)

    def _on_sigint(self, *_):
        rospy.logwarn('interrupted, stop')
        try:
            self._stop()
        finally:
            rospy.signal_shutdown('sigint')
            sys.exit(0)

    def _drive_to_point(self, tx, ty, hold_yaw, timeout=STEP_TIMEOUT):
        rate = rospy.Rate(RATE_HZ)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            cur = self._cur()
            if cur is None:
                self._stop(); return False
            ex = tx - cur[0]
            ey = ty - cur[1]
            dist = math.hypot(ex, ey)
            if dist < POS_TOL:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn('    drive_to (%.2f,%.2f) TIMEOUT dist=%.3f', tx, ty, dist)
                self._stop(); return False
            if abs(ex) < abs(ey):
                vx_map = _clamp(K_LIN_HOLD * ex, -V_LIN, V_LIN)
                vy_map = _clamp(K_LIN * ey, -V_LIN, V_LIN)
            else:
                vx_map = _clamp(K_LIN * ex, -V_LIN, V_LIN)
                vy_map = _clamp(K_LIN_HOLD * ey, -V_LIN, V_LIN)
            sp = math.hypot(vx_map, vy_map)
            if 1e-6 < sp < V_LIN_MIN:
                vx_map *= V_LIN_MIN / sp
                vy_map *= V_LIN_MIN / sp
            th = cur[2]
            c = math.cos(th); s = math.sin(th)
            tw = Twist()
            tw.linear.x =  c * vx_map + s * vy_map   # map->body 旋转 -th
            tw.linear.y = -s * vx_map + c * vy_map
            eyaw = _norm(hold_yaw - th)
            if abs(eyaw) > 0.02:
                wz = _clamp(K_YAW * eyaw, -V_ROT, V_ROT)
                if abs(wz) < V_ROT_MIN:
                    wz = math.copysign(V_ROT_MIN, wz)
                tw.angular.z = wz
            self._pub.publish(tw)
            rate.sleep()
        self._stop()
        return True

    def _rotate_to(self, target_yaw, timeout=STEP_TIMEOUT):
        rate = rospy.Rate(RATE_HZ)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            cur = self._cur()
            if cur is None:
                self._stop(); return False
            e = _norm(target_yaw - cur[2])
            if abs(e) < YAW_TOL:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn('    rotate_to %.2f TIMEOUT err=%.3f', target_yaw, e)
                self._stop(); return False
            wz = _clamp(K_YAW * e, -V_ROT, V_ROT)
            if abs(wz) < V_ROT_MIN:
                wz = math.copysign(V_ROT_MIN, wz)
            tw = Twist()
            tw.angular.z = wz
            self._pub.publish(tw)
            rate.sleep()
        self._stop()
        return True

    def goto(self, x, y, yaw):
        cur = self._cur()
        if cur is None:
            rospy.logerr('no tf pose, abort goto')
            return False
        hold = cur[2]
        legs = [(x, cur[1]), (x, y)]   # 先 X 后 Y, 全程锁初始航向
        for i, (gx, gy) in enumerate(legs):
            rospy.loginfo('  step%d -> (%.3f, %.3f) hold_yaw=%.2f', i + 1, gx, gy, hold)
            if not self._drive_to_point(gx, gy, hold):
                return False
        rospy.loginfo('  step3 -> rotate %.2f', yaw)
        return self._rotate_to(yaw)

    def run(self):
        rospy.loginfo('seven_point_test: %d points', len(WAYPOINTS))
        for i, (x, y, yaw) in enumerate(WAYPOINTS):
            rospy.loginfo('[%d/%d] -> (%.3f, %.3f, %.2f)', i + 1, len(WAYPOINTS), x, y, yaw)
            if not self.goto(x, y, yaw):
                rospy.logerr('point %d FAILED, abort', i + 1)
                break
            rospy.loginfo('point %d reached', i + 1)
        self._stop()
        rospy.loginfo('seven_point_test done')


if __name__ == '__main__':
    SevenPointTest().run()
