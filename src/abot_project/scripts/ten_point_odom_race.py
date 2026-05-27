#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
十点纯里程计比赛保底脚本（不依赖 amcl/雷达/SLAM）。

位姿只读 odom->base_footprint tf（bringup 的 EKF 一直发布），定位栈全挂也能跑，
故为"保底"。每点 L 形分解直达 + 锁航向 + 只沿 xy 轴走（满足"碰挡板即判负"硬约束）。
路线在 x=2.4 走廊里走, 仅darting到 x=3.6 取远点, 规避中间挡板。

坐标系 = odom（开机/重启 bringup 时归零）。因此:
  ★ 车必须物理摆在【比赛原点】、车头朝 +x 后再起 bringup，使 odom 原点=比赛原点。
  ★ 纯里程计无回环校正, 长距离有累积漂移; 这是 SLAM/amcl 不可用时的兜底方案。

前置(手动启动)：
  roslaunch abot bringup.launch        # 只需这一个! 不要起 localize/navigate/雷达

用法：python2 ten_point_odom_race.py   (或 rosrun abot ten_point_odom_race.py)
中断(Ctrl-C)：立即停车。
"""

import sys
import math
import signal

import rospy
import tf
from geometry_msgs.msg import Twist


# 10 个航点 (odom 系 x, y, yaw[rad])。P1~P7=比赛任务点, 其余=走廊过渡点
WAYPOINTS = [
    (2.4, 0.6, 0.0),   # 进入走廊
    (3.6, 0.6, 0.0),   # Point 1
    (2.4, 0.6, 0.0),   # Point 2
    (2.4, 1.8, 0.0),   # 走廊上移
    (3.6, 1.8, 0.0),   # Point 3
    (2.4, 1.8, 0.0),   # Point 4
    (2.4, 2.6, 0.0),   # 走廊上移
    (3.6, 2.6, 0.0),   # Point 5
    (2.4, 2.6, 0.0),   # Point 6
    (2.4, 2.0, 0.0),   # Point 7 (终点/回收)
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
STEP_TIMEOUT = 30.0  # s    单步超时(fail-fast, 防 30s 不动判负)

BASE_FRAME = 'base_footprint'
ODOM_FRAME = 'odom'      # ★纯里程计: 用 odom 而非 map, 不需 amcl


def _norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TenPointOdomRace(object):
    def __init__(self):
        rospy.init_node('ten_point_odom_race')
        self._tf = tf.TransformListener()
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        rospy.loginfo('waiting tf %s->%s ...', ODOM_FRAME, BASE_FRAME)
        try:
            self._tf.waitForTransform(ODOM_FRAME, BASE_FRAME, rospy.Time(0), rospy.Duration(10.0))
        except tf.Exception:
            rospy.logwarn('no %s->%s tf, 确认 bringup.launch (EKF) 已启动', ODOM_FRAME, BASE_FRAME)

        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigint)
        rospy.sleep(0.5)

    def _cur(self):
        try:
            trans, rot = self._tf.lookupTransform(ODOM_FRAME, BASE_FRAME, rospy.Time(0))
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
                vx_odom = _clamp(K_LIN_HOLD * ex, -V_LIN, V_LIN)
                vy_odom = _clamp(K_LIN * ey, -V_LIN, V_LIN)
            else:
                vx_odom = _clamp(K_LIN * ex, -V_LIN, V_LIN)
                vy_odom = _clamp(K_LIN_HOLD * ey, -V_LIN, V_LIN)
            sp = math.hypot(vx_odom, vy_odom)
            if 1e-6 < sp < V_LIN_MIN:
                vx_odom *= V_LIN_MIN / sp
                vy_odom *= V_LIN_MIN / sp
            th = cur[2]
            c = math.cos(th); s = math.sin(th)
            tw = Twist()
            tw.linear.x =  c * vx_odom + s * vy_odom   # odom->body 旋转 -th
            tw.linear.y = -s * vx_odom + c * vy_odom
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
        rospy.loginfo('ten_point_odom_race: %d points (pure odometry)', len(WAYPOINTS))
        for i, (x, y, yaw) in enumerate(WAYPOINTS):
            rospy.loginfo('[%d/%d] -> (%.3f, %.3f, %.2f)', i + 1, len(WAYPOINTS), x, y, yaw)
            if not self.goto(x, y, yaw):
                rospy.logerr('waypoint %d FAILED, abort', i + 1)
                break
            rospy.loginfo('waypoint %d reached', i + 1)
        self._stop()
        rospy.loginfo('ten_point_odom_race done')


if __name__ == '__main__':
    TenPointOdomRace().run()
