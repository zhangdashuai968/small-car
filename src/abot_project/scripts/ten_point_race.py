#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
基础十点比赛脚本（localize.launch 版：amcl/map 定位 + 抓放，绕开 move_base）。

10 点路线：去每个抓取点(x=3.6)前，先经过对应放置点(x=2.4, 同 y)做走廊过渡，
darting 出去抓 -> 退回放，规避中间挡板。位姿读 tf map->base_footprint(amcl+EKF)。
执行用锁航向 cmd_vel 原语(只沿 xy 轴、原地转), 抓放沿用 auto_task_runner 的握手。

路线(map 系):
  进走廊(2.4,0.6) -> P1抓(3.6,0.6) -> P2放(2.4,0.6)
  上移  (2.4,1.8) -> P3抓(3.6,1.8) -> P4放(2.4,1.8)
  上移  (2.4,2.6) -> P5抓(3.6,2.6) -> P6放(2.4,2.6) -> 终点(2.4,2.0)

前置(手动启动)：
  roslaunch abot bringup.launch
  roslaunch abot localize.launch         # map_server + amcl, 提供 map->base_footprint
  roslaunch ZachLab_grasp grasp.launch   # 抓取需要
  roslaunch vl_locate vl_locate.launch   # conda 39, 抓取需要

★ 赛前把下方 WAYPOINTS 里 P1/P3/P5 的 object 改成裁判公布的物品关键词。

用法：python2 ten_point_race.py   (或 rosrun abot ten_point_race.py)
中断(Ctrl-C)：停车 + 舵机回 home + /grab=False。
"""

import sys
import math
import signal
import time

import rospy
import tf
from geometry_msgs.msg import Twist
from riki_msgs.msg import Servo
from vl_locate.srv import GetObject


# 10 点路线 (map 系 x, y, yaw[rad], action, object)。action: grab/place/pass
WAYPOINTS = [
    (2.4, 0.6, 0.0, 'pass',  ''),           # 进走廊(过渡)
    (3.6, 0.6, 0.0, 'grab',  '物体1'),       # Point 1 抓
    (2.4, 0.6, 0.0, 'place', '图片中心点'),   # Point 2 放
    (2.4, 1.8, 0.0, 'pass',  ''),           # 上移(过渡)
    (3.6, 1.8, 0.0, 'grab',  '物体2'),       # Point 3 抓
    (2.4, 1.8, 0.0, 'place', '图片中心点'),   # Point 4 放
    (2.4, 2.6, 0.0, 'pass',  ''),           # 上移(过渡)
    (3.6, 2.6, 0.0, 'grab',  '物体3'),       # Point 5 抓
    (2.4, 2.6, 0.0, 'place', '图片中心点'),   # Point 6 放
    (2.4, 2.0, 0.0, 'pass',  ''),           # Point 7 终点
]

SERVO_HOME = (88, 20)
GRASP_WAIT_SEC = 25

# --- 原语控制器参数 (与 auto_task_runner 一致) ---
V_LIN      = 0.20
V_ROT      = 0.40
V_LIN_MIN  = 0.04
V_ROT_MIN  = 0.10
K_LIN      = 4.0
K_LIN_HOLD = 5.0
K_YAW      = 1.5
POS_TOL    = 0.04
YAW_TOL    = 0.03
RATE_HZ    = 20
STEP_TIMEOUT = 30.0

MAP_FRAME  = 'map'
BASE_FRAME = 'base_footprint'


def _norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TenPointRace(object):
    def __init__(self):
        rospy.init_node('ten_point_race')
        self._tf = tf.TransformListener()
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.servo_pub = rospy.Publisher('servo', Servo, queue_size=1, latch=True)
        self.vlm = None   # 延迟加载

        rospy.loginfo('waiting tf %s->%s ...', MAP_FRAME, BASE_FRAME)
        try:
            self._tf.waitForTransform(MAP_FRAME, BASE_FRAME, rospy.Time(0), rospy.Duration(10.0))
        except tf.Exception:
            rospy.logwarn('no %s->%s tf, 确认 localize.launch 已启动且 amcl 收敛', MAP_FRAME, BASE_FRAME)

        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigint)
        self.refresh()

    # ---------- 状态 ----------
    def _cur(self):
        try:
            trans, rot = self._tf.lookupTransform(MAP_FRAME, BASE_FRAME, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return (trans[0], trans[1], yaw)

    def refresh(self):
        self._stop()
        self._servo_home()
        rospy.set_param('/grab', False)
        time.sleep(1.0)

    def _servo_home(self):
        m = Servo()
        m.Servo1, m.Servo2 = SERVO_HOME
        self.servo_pub.publish(m)

    def _on_sigint(self, *_):
        rospy.logwarn('interrupted, stop + servo home')
        try:
            self._stop()
            self._servo_home()
        finally:
            rospy.signal_shutdown('sigint')
            sys.exit(0)

    # ---------- 原语 ----------
    def _stop(self):
        t = Twist()
        for _ in range(3):
            self._pub.publish(t)
            rospy.sleep(0.02)

    def _drive_to_point(self, tx, ty, hold_yaw, timeout=STEP_TIMEOUT):
        rate = rospy.Rate(RATE_HZ)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            cur = self._cur()
            if cur is None:
                self._stop(); return False
            ex = tx - cur[0]
            ey = ty - cur[1]
            if math.hypot(ex, ey) < POS_TOL:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn('    drive_to (%.2f,%.2f) TIMEOUT', tx, ty)
                self._stop(); return False
            if abs(ex) < abs(ey):
                vx = _clamp(K_LIN_HOLD * ex, -V_LIN, V_LIN)
                vy = _clamp(K_LIN * ey, -V_LIN, V_LIN)
            else:
                vx = _clamp(K_LIN * ex, -V_LIN, V_LIN)
                vy = _clamp(K_LIN_HOLD * ey, -V_LIN, V_LIN)
            sp = math.hypot(vx, vy)
            if 1e-6 < sp < V_LIN_MIN:
                vx *= V_LIN_MIN / sp
                vy *= V_LIN_MIN / sp
            th = cur[2]
            c = math.cos(th); s = math.sin(th)
            tw = Twist()
            tw.linear.x =  c * vx + s * vy
            tw.linear.y = -s * vx + c * vy
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
                rospy.logwarn('    rotate_to %.2f TIMEOUT', target_yaw)
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

    # ---------- 抓放 ----------
    def do_action(self, action, obj):
        if action == 'pass':
            return True
        if self.vlm is None:
            rospy.loginfo('waiting /vlm_detection...')
            rospy.wait_for_service('/vlm_detection')
            self.vlm = rospy.ServiceProxy('/vlm_detection', GetObject)
        rospy.set_param('/grab', action == 'grab')
        try:
            self.vlm(obj)
        except rospy.ServiceException as e:
            rospy.logerr('vlm call failed: %s' % e)
            return False
        rospy.sleep(GRASP_WAIT_SEC)
        return True

    def run(self):
        rospy.loginfo('ten_point_race: %d points', len(WAYPOINTS))
        for i, (x, y, yaw, action, obj) in enumerate(WAYPOINTS):
            rospy.loginfo('[%d/%d] -> (%.3f, %.3f, %.2f) action=%s %s',
                          i + 1, len(WAYPOINTS), x, y, yaw, action, obj)
            if not self.goto(x, y, yaw):
                rospy.logerr('goto wp%d failed, skip', i + 1)
                continue
            if not self.do_action(action, obj):
                rospy.logerr('action on wp%d failed, skip', i + 1)
                continue
        rospy.loginfo('all waypoints done')
        self._stop()
        self._servo_home()


if __name__ == '__main__':
    TenPointRace().run()
