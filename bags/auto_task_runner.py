#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
多点位顺序直达(L形)分解 + 原地转 + 抓/放。
直接发 cmd_vel 原语控制，不依赖 move_base/TEB/DWA，因此与 NAV_PATH(teb/dwa) 无关。
每个目标点位分解为一直线 + 另一直线(全程保持车头朝向不漂) + 最后原地转到目标朝向。
位姿来源 tf map->base_footprint(世界坐标系)，由 amcl+EKF 提供。

前置(手动启动)：
  roscore
  roslaunch abot bringup.launch
  roslaunch abot navigate.launch     # 只需提供 amcl/map/laser 做定位，move_base 不启用
  roslaunch ZachLab_grasp grasp.launch    # 抓取时才需要
  roslaunch vl_locate vl_locate.launch    # conda 39, 抓取时才需要

用法：python2 auto_task_runner.py [waypoints.yaml]
中断(Ctrl-C)/异常或刷新：停止后回舵机 home，/grab=False
"""

import os
import sys
import math
import signal
import time

import rospy
import yaml
import tf

from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import LaserScan

try:
    from riki_msgs.msg import Servo
    HAS_SERVO = True
except ImportError:
    Servo = None
    HAS_SERVO = False

try:
    from vl_locate.srv import GetObject
    HAS_VLM = True
except ImportError:
    GetObject = None
    HAS_VLM = False


SERVO_HOME = (88, 20)
GRASP_WAIT_SEC = 25

# --- 原语控制器参数(可调) ---
V_LIN     = 0.20   # m/s   直线/横移线速度上限
V_ROT     = 0.40   # rad/s 原地转角速度上限
V_LIN_MIN = 0.04   # m/s   最小线速度(克服静摩擦)
V_ROT_MIN = 0.10   # rad/s 最小转角速度
K_LIN     = 4.0    # 线速度 P 增益
K_LIN_HOLD = 5.0   # 非主运动轴P增益(压制漂移)
K_YAW     = 1.5    # 角速度 P 增益
POS_TOL   = 0.04   # m     终点位置容差
YAW_TOL   = 0.03   # rad   终点角度容差(~1.7°)
RATE_HZ   = 20
STEP_TIMEOUT = 30.0  # s    单步超时(fail-fast)
FAST_RATIO = 0.8    # 快速路阈值: 速度 >= 80% V_LIN 算快速路

BASE_FRAME = 'base_footprint'
MAP_FRAME  = 'map'


def _norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TaskRunner(object):
    def __init__(self, waypoints):
        rospy.init_node('auto_task_runner')
        self.waypoints = waypoints
        self.axis_order = rospy.get_param('~axis_order', 'xy')  # 'xy'=先X后Y, 'yx'=先Y后X

        self._tf = tf.TransformListener()
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._mb_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.servo_pub = rospy.Publisher('servo', Servo, queue_size=1, latch=True) if HAS_SERVO else None
        self.vlm = None  # 延迟加载，避免依赖完整抓取栈

        rospy.loginfo('waiting tf %s->%s ...', MAP_FRAME, BASE_FRAME)
        try:
            self._tf.waitForTransform(MAP_FRAME, BASE_FRAME, rospy.Time(0), rospy.Duration(10.0))
        except tf.Exception:
            rospy.logwarn('no %s->%s tf, 确认 navigate.launch 已启动 amcl 已收敛', MAP_FRAME, BASE_FRAME)

        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigint)
        time.sleep(0.5)
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
        rospy.loginfo('--- refresh state ---')
        self._stop()
        self._mb_cancel.publish(GoalID())   # 取消任何残留 move_base goal，释放 cmd_vel
        self._servo_home()
        rospy.set_param('/grab', False)
        time.sleep(1.0)

    def _servo_home(self):
        if not HAS_SERVO:
            return
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

    def _drive_to_point(self, tx, ty, hold_yaw, pos_tol=POS_TOL, timeout=STEP_TIMEOUT):
        """闭环直行到 map 下 (tx,ty)，全程锁 hold_yaw。
        返回 (ok, fast_s, fine_s) — fast=速度≥80%V_LIN, fine=速度<80%V_LIN"""
        rate = rospy.Rate(RATE_HZ)
        t0 = rospy.Time.now()
        t_last = t0
        t_fast = 0.0
        t_fine = 0.0
        while not rospy.is_shutdown():
            cur = self._cur()
            if cur is None:
                self._stop(); return (False, 0, 0)
            ex = tx - cur[0]
            ey = ty - cur[1]
            dist = math.hypot(ex, ey)
            if dist < pos_tol:
                rospy.loginfo('    arrived (%.3f,%.3f) yaw=%.2f target=(%.3f,%.3f) err=(%.3f,%.3f) dist=%.3f', cur[0], cur[1], cur[2], tx, ty, ex, ey, dist)
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn('    drive_to (%.2f,%.2f) TIMEOUT dist=%.3f', tx, ty, dist)
                self._stop(); return (False, 0, 0)
            # 非主运动轴用更高增益压制漂移
            if abs(ex) < abs(ey):
                vx_map = _clamp(K_LIN_HOLD * ex, -V_LIN, V_LIN)
                vy_map = _clamp(K_LIN * ey, -V_LIN, V_LIN)
            else:
                vx_map = _clamp(K_LIN * ex, -V_LIN, V_LIN)
                vy_map = _clamp(K_LIN_HOLD * ey, -V_LIN, V_LIN)
            sp = math.hypot(vx_map, vy_map)
            if 1e-6 < sp < V_LIN_MIN:        # 克服静摩擦，保持最小速度，不反转
                vx_map *= V_LIN_MIN / sp
                vy_map *= V_LIN_MIN / sp
            th = cur[2]
            c = math.cos(th); s = math.sin(th)
            tw = Twist()
            tw.linear.x =  c * vx_map + s * vy_map   # map->body 旋转 -th
            tw.linear.y = -s * vx_map + c * vy_map
            eyaw = _norm(hold_yaw - th)
            if abs(eyaw) > 0.02:                     # 抑制航向漂移
                wz = _clamp(K_YAW * eyaw, -V_ROT, V_ROT)
                if abs(wz) < V_ROT_MIN:
                    wz = math.copysign(V_ROT_MIN, wz)
                tw.angular.z = wz
            # 计时: body系合速度 >= 80% V_LIN 算快速路
            now = rospy.Time.now()
            dt = (now - t_last).to_sec()
            body_sp = math.hypot(tw.linear.x, tw.linear.y)
            if body_sp >= FAST_RATIO * V_LIN:
                t_fast += dt
            else:
                t_fine += dt
            t_last = now
            self._pub.publish(tw)
            rate.sleep()
        self._stop()
        return (True, t_fast, t_fine)

    def _rotate_to(self, target_yaw, yaw_tol=YAW_TOL, timeout=STEP_TIMEOUT):
        """返回 (ok, rot_s)"""
        rate = rospy.Rate(RATE_HZ)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            cur = self._cur()
            if cur is None:
                self._stop(); return (False, 0)
            e = _norm(target_yaw - cur[2])
            if abs(e) < yaw_tol:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn('    rotate_to %.2f TIMEOUT err=%.3f', target_yaw, e)
                self._stop(); return (False, 0)
            wz = _clamp(K_YAW * e, -V_ROT, V_ROT)
            if abs(wz) < V_ROT_MIN:
                wz = math.copysign(V_ROT_MIN, wz)
            tw = Twist()
            tw.angular.z = wz
            self._pub.publish(tw)
            rate.sleep()
        self._stop()
        return (True, (rospy.Time.now() - t0).to_sec())

    def goto(self, x, y, yaw, frame='map'):
        t0 = rospy.Time.now()
        t_fast = 0.0
        t_fine = 0.0
        t_rot = 0.0
        cur = self._cur()
        if cur is None:
            rospy.logerr('no tf pose, abort goto')
            return False
        hold = cur[2]
        if frame == 'body':
            c = math.cos(hold); s = math.sin(hold)
            fwd_x = cur[0] + x * c
            fwd_y = cur[1] + x * s
            # leg1: body-x (forward/back)
            rospy.loginfo('  step1 -> (%.3f, %.3f) hold_yaw=%.2f', fwd_x, fwd_y, hold)
            ok, f1, d1 = self._drive_to_point(fwd_x, fwd_y, hold)
            if not ok:
                return False
            t_fast += f1; t_fine += d1
            # re-read yaw after leg1, recompute leg2 target
            cur = self._cur()
            if cur is None:
                rospy.logerr('no tf pose after leg1, abort goto')
                return False
            hold = cur[2]
            c = math.cos(hold); s = math.sin(hold)
            tx = cur[0] + y * (-s)
            ty = cur[1] + y * c
            rospy.loginfo('  step2 -> (%.3f, %.3f) hold_yaw=%.2f', tx, ty, hold)
            ok, f2, d2 = self._drive_to_point(tx, ty, hold)
            if not ok:
                return False
            t_fast += f2; t_fine += d2
        else:
            # map frame: L-shape (X first, then Y) for Mecanum stability
            rospy.loginfo('  leg1 X -> (%.3f, %.3f) hold_yaw=%.2f', x, cur[1], hold)
            ok, f1, d1 = self._drive_to_point(x, cur[1], hold)
            if not ok:
                return False
            t_fast += f1; t_fine += d1
            cur = self._cur()
            if cur is None:
                rospy.logerr('no tf pose after leg1, abort goto')
                return False
            hold = cur[2]
            rospy.loginfo('  leg2 Y -> (%.3f, %.3f) hold_yaw=%.2f', x, y, hold)
            ok, f2, d2 = self._drive_to_point(x, y, hold)
            if not ok:
                return False
            t_fast += f2; t_fine += d2
        rospy.loginfo('  final yaw -> %.2f', yaw)
        ok, t_rot = self._rotate_to(yaw)
        if not ok:
            return False
        t_total = (rospy.Time.now() - t0).to_sec()
        t_prep = t_total - t_fast - t_fine - t_rot
        rospy.loginfo('  [STAGE] 准备=%.2fs 快速路=%.2fs 微调=%.2fs(减速%.2fs+旋转%.2fs) 总计=%.2fs',
                      t_prep, t_fast, t_fine + t_rot, t_fine, t_rot, t_total)
        return True
    def do_action(self, action, obj):
        if action == 'pass':
            return True
        if action == 'wait':
            duration = float(obj) if obj else 0
            if duration > 0:
                rospy.loginfo('  waiting %.1fs ...', duration)
                rospy.sleep(duration)
            return True
        if not HAS_VLM:
            rospy.logwarn('vl_locate not installed, skip action=%s', action)
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
        rospy.sleep(GRASP_WAIT_SEC)   # TODO: 改成订阅 /success 做实闭环信号
        return True

    def run(self):
        for i, wp in enumerate(self.waypoints):
            name = wp.get('name', 'wp%d' % i)
            rospy.loginfo('[%d/%d] %s -> (%.3f, %.3f, %.2f) action=%s',
                          i + 1, len(self.waypoints), name,
                          wp['x'], wp['y'], wp['yaw'], wp['action'])
            if not self.goto(wp['x'], wp['y'], wp['yaw'], wp.get('frame', 'map')):
                rospy.logerr('goto %s failed, skip', name)
                continue
            if not self.do_action(wp['action'], wp.get('object', '')):
                rospy.logerr('action on %s failed, skip', name)
                continue
        rospy.loginfo('all waypoints done')
        self._stop()
        self._servo_home()


def load_waypoints(path):
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)
    return cfg['waypoints']


if __name__ == '__main__':
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'waypoints.yaml')
    wps = load_waypoints(cfg_path)
    TaskRunner(wps).run()