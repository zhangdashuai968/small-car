#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
���λ����������ֱ��(L��)���� + ԭ��ת + ץ/�š�
ֱ�ӷ� cmd_vel ԭ����ƣ������� move_base/TEB/DWA���������� NAV_PATH(teb/dwa) �޹ء�
ÿ����λ������һ��ֱ�� �� ������һ��ֱ��(ȫ��������������Ư) �� ���ԭ��ת��Ŀ�곯��
λ�˷����� tf map->base_footprint(��������Ƶ)���� amcl+EKF �ṩ��

ǰ�ã��ֶ������ã���
  roscore
  roslaunch abot bringup.launch
  roslaunch abot navigate.launch     # ֻ������ amcl/map/laser ����λ��move_base ���ü���
  roslaunch ZachLab_grasp grasp.launch    # ץȡʱ����Ҫ
  roslaunch vl_locate vl_locate.launch    # conda 39, ץȡʱ����Ҫ

�ܣ�python2 auto_task_runner.py [waypoints.yaml]
�ж�(Ctrl-C)/���ܻ�ˢ�£�ͣ������λ��� home��/grab=False
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
from riki_msgs.msg import Servo
from vl_locate.srv import GetObject


SERVO_HOME = (88, 20)
GRASP_WAIT_SEC = 25

# --- ԭ����Ʋ���(�����) ---
V_LIN     = 0.20   # m/s   ֱ��/�������ٶ�����
V_ROT     = 0.40   # rad/s ԭ��ת���ٶ�����
V_LIN_MIN = 0.04   # m/s   ��С���ٶ�(�˷���Ħ��)
V_ROT_MIN = 0.10   # rad/s ��Сת��
K_LIN     = 4.0    #���ٶ� P ����
K_LIN_HOLD = 5.0   # 非主运动轴P增益(压制漂移)
K_YAW     = 1.5    #���� P ����
POS_TOL   = 0.04   # m     ��λ�ݲ�
YAW_TOL   = 0.03   # rad   �����ݲ�(~1.7��)
RATE_HZ   = 20
STEP_TIMEOUT = 30.0  # s    ������ʱ(fail-fast)

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
        self.axis_order = rospy.get_param('~axis_order', 'xy')  # 'xy'=��X��Y, 'yx'=��Y��X

        self._tf = tf.TransformListener()
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._mb_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.servo_pub = rospy.Publisher('servo', Servo, queue_size=1, latch=True)
        self.vlm = None  # �����أ�������������ץȡջ

        rospy.loginfo('waiting tf %s->%s ...', MAP_FRAME, BASE_FRAME)
        try:
            self._tf.waitForTransform(MAP_FRAME, BASE_FRAME, rospy.Time(0), rospy.Duration(10.0))
        except tf.Exception:
            rospy.logwarn('no %s->%s tf, ȷ�� navigate.launch ������ amcl ������', MAP_FRAME, BASE_FRAME)

        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigint)
        time.sleep(0.5)
        self.refresh()

    # ---------- ״̬ ----------
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
        self._mb_cancel.publish(GoalID())   # ȡ���κβ��� move_base goal�������� cmd_vel
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

    # ---------- ԭ�� ----------
    def _stop(self):
        t = Twist()
        for _ in range(3):
            self._pub.publish(t)
            rospy.sleep(0.02)

    def _drive_to_point(self, tx, ty, hold_yaw, pos_tol=POS_TOL, timeout=STEP_TIMEOUT):
        """�ջ�ֱ�е� map �� (tx,ty)��ȫ���� hold_yaw��λ�Ƶ���ʱ���ظ���ֱ�ߡ�"""
        rate = rospy.Rate(RATE_HZ)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            cur = self._cur()
            if cur is None:
                self._stop(); return False
            ex = tx - cur[0]
            ey = ty - cur[1]
            dist = math.hypot(ex, ey)
            if dist < pos_tol:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn('    drive_to (%.2f,%.2f) TIMEOUT dist=%.3f', tx, ty, dist)
                self._stop(); return False
            # 非主运动轴用更高增益压制漂移
            if abs(ex) < abs(ey):
                vx_map = _clamp(K_LIN_HOLD * ex, -V_LIN, V_LIN)
                vy_map = _clamp(K_LIN * ey, -V_LIN, V_LIN)
            else:
                vx_map = _clamp(K_LIN * ex, -V_LIN, V_LIN)
                vy_map = _clamp(K_LIN_HOLD * ey, -V_LIN, V_LIN)
            sp = math.hypot(vx_map, vy_map)
            if 1e-6 < sp < V_LIN_MIN:        # �˷���Ħ������С���ٶȣ����ַ���
                vx_map *= V_LIN_MIN / sp
                vy_map *= V_LIN_MIN / sp
            th = cur[2]
            c = math.cos(th); s = math.sin(th)
            tw = Twist()
            tw.linear.x =  c * vx_map + s * vy_map   # map->body ��ת -th
            tw.linear.y = -s * vx_map + c * vy_map
            eyaw = _norm(hold_yaw - th)
            if abs(eyaw) > 0.02:                     # ����������Ư��
                wz = _clamp(K_YAW * eyaw, -V_ROT, V_ROT)
                if abs(wz) < V_ROT_MIN:
                    wz = math.copysign(V_ROT_MIN, wz)
                tw.angular.z = wz
            self._pub.publish(tw)
            rate.sleep()
        self._stop()
        return True

    def _rotate_to(self, target_yaw, yaw_tol=YAW_TOL, timeout=STEP_TIMEOUT):
        rate = rospy.Rate(RATE_HZ)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            cur = self._cur()
            if cur is None:
                self._stop(); return False
            e = _norm(target_yaw - cur[2])
            if abs(e) < yaw_tol:
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

    def goto(self, x, y, yaw, frame='map'):
        cur = self._cur()
        if cur is None:
            rospy.logerr('no tf pose, abort goto')
            return False
        hold = cur[2]
        if frame == 'body':
            c = math.cos(hold); s = math.sin(hold)
            fwd_x = cur[0] + x * c
            fwd_y = cur[1] + x * s
            tx = fwd_x + y * (-s)
            ty = fwd_y + y * c
            legs = [(fwd_x, fwd_y), (tx, ty)]
        elif self.axis_order == 'yx':
            legs = [(cur[0], y), (x, y)]
        else:
            legs = [(x, cur[1]), (x, y)]
        for i, (gx, gy) in enumerate(legs):
            rospy.loginfo('  step%d -> (%.3f, %.3f) hold_yaw=%.2f', i + 1, gx, gy, hold)
            if not self._drive_to_point(gx, gy, hold):
                return False
        rospy.loginfo('  step3 -> %.2f', yaw)
        return self._rotate_to(yaw)

    # ---------- ץ�� ----------
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
        rospy.sleep(GRASP_WAIT_SEC)   # TODO: �Ķ��� /success ��ʵ����ź�
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

