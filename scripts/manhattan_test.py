#!/usr/bin/env python
# -*- coding: utf-8 -*-
# L 形(曼哈顿)开环定时测试: 全向平移, 车身不转。
# 后退 dist (车体 -x) -> 停 -> 右移 dist (车体 -y) -> 停, 全程 angular.z=0。
# 梯形速度剖面(平滑加减速), 积分位移精确=dist。订阅 /odom 记录起止位姿,
# 走完把位移投影到"起始车体系", 打印实测 vs 理论(-dist,-dist,0), 看航向漂/串轴。
# 用法: python manhattan_test.py _dist:=2.0 _v:=0.4
# 前置: 只起 roslaunch abot bringup.launch (提供 /cmd_vel + /odom), 不起 navigate。
from __future__ import print_function
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "got": False}


def _odom_cb(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    _pose["x"] = p.x
    _pose["y"] = p.y
    _pose["yaw"] = yaw
    _pose["got"] = True


def _norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    rospy.init_node("manhattan_test")
    dist = float(rospy.get_param("~dist", 2.0))
    d_back = float(rospy.get_param("~back", dist))
    d_right = float(rospy.get_param("~right", dist))
    v = min(float(rospy.get_param("~v", 0.4)), 0.4)   # 沿用 0.4 硬上限
    ramp = float(rospy.get_param("~ramp", 0.5))
    hz = float(rospy.get_param("~rate", 50.0))
    topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
    odom_topic = rospy.get_param("~odom_topic", "/odom")
    hold = rospy.get_param("~hold_yaw", True)   # 平移时航向闭环锁定
    kyaw = float(rospy.get_param("~kyaw", 1.5))
    max_wz = float(rospy.get_param("~max_wz", 0.5))

    pub = rospy.Publisher(topic, Twist, queue_size=10)
    rospy.Subscriber(odom_topic, Odometry, _odom_cb)
    rate = rospy.Rate(hz)

    def stop(n=12):
        t = Twist()
        for _ in range(n):
            pub.publish(t)
            rospy.sleep(0.02)

    rospy.on_shutdown(lambda: stop(8))

    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and not _pose["got"]:
        if rospy.Time.now().to_sec() - t0 > 5.0:
            rospy.logwarn("5s 没收到 %s, 确认 bringup 在跑", odom_topic)
            return
        rate.sleep()

    def snap():
        return (_pose["x"], _pose["y"], _pose["yaw"])

    start = snap()
    rospy.loginfo("start odom: x=%.3f y=%.3f yaw=%.1f deg",
                  start[0], start[1], math.degrees(start[2]))

    # 单段梯形剖面: axis 'x'/'y', sign +/-1, 距离 D。积分位移精确=D。
    def run_seg(axis, sign, D):
        if D < v * ramp:                      # 太短退化为三角(此处 D=2 不会触发)
            a = v / ramp
            ta = math.sqrt(D / a)
            vp = a * ta
            tc = 0.0
        else:
            ta = ramp
            vp = v
            tc = (D - v * ramp) / v
        T = 2 * ta + tc
        ts = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            el = rospy.Time.now().to_sec() - ts
            if el >= T:
                break
            if el < ta:
                sp = vp * el / ta
            elif el < ta + tc:
                sp = vp
            else:
                sp = vp * (T - el) / ta
            cmd = Twist()
            if axis == 'x':
                cmd.linear.x = sign * sp
            else:
                cmd.linear.y = sign * sp
            if hold:
                eyaw = _norm(start[2] - _pose["yaw"])
                cmd.angular.z = max(-max_wz, min(max_wz, kyaw * eyaw))
            else:
                cmd.angular.z = 0.0
            pub.publish(cmd)
            rate.sleep()
        stop(10)

    rospy.loginfo(">> seg1 后退 %.2fm (vx<0)", d_back)
    run_seg('x', -1.0, d_back)
    rospy.sleep(1.0)
    p1 = snap()

    rospy.loginfo(">> seg2 右移 %.2fm (vy<0)", d_right)
    run_seg('y', -1.0, d_right)
    rospy.sleep(1.0)
    p2 = snap()

    def to_body(p):
        dx = p[0] - start[0]
        dy = p[1] - start[1]
        c = math.cos(start[2])
        s = math.sin(start[2])
        return (dx * c + dy * s, -dx * s + dy * c,
                math.degrees(_norm(p[2] - start[2])))

    b1 = to_body(p1)
    b2 = to_body(p2)
    rospy.loginfo("=== 结果 (起始车体系) ===")
    rospy.loginfo("seg1后 实测 dx=%.3f dy=%.3f dyaw=%.1f | 理论 (-%.2f, 0, 0)",
                  b1[0], b1[1], b1[2], d_back)
    rospy.loginfo("seg2后 实测 dx=%.3f dy=%.3f dyaw=%.1f | 理论 (-%.2f, -%.2f, 0)",
                  b2[0], b2[1], b2[2], d_back, d_right)
    rospy.loginfo("终点航向漂 %.1f deg; 串轴: seg1横向漂 %.3f m, seg2前漂 %.3f m",
                  b2[2], b1[1], b2[0] - b1[0])
    stop(10)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
