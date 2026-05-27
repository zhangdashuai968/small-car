#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 一次性圆周轨迹测试: 开环发 /cmd_vel, 不过 move_base/teb, 不受 planner 软限.
# 用法:
#   python circle_run.py _mode:=diff _v:=0.4          # 差速转向, 先低速验证
#   python circle_run.py _mode:=omni _v:=0.6 _laps:=3 # 全向平移
# 参数(私有 rosparam):
#   _mode  diff|omni   差速转向 / 全向平移        默认 diff
#   _v     m/s         切向线速度, 硬上限 0.4       默认 0.4
#   _R     m           半径                        默认 1.5
#   _laps  圈           跑满后自停                  默认 3
#   _dir   +1|-1        +1 逆时针/左转, -1 顺时针   默认 +1
#   _ramp  s           起步加速渐进, 防麦轮打滑     默认 1.5
#   _rate  Hz          发布频率                    默认 50
#   _cmd_vel_topic     默认 /cmd_vel
from __future__ import print_function
import math
import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("circle_run")
    V_MAX = 0.4
    mode = rospy.get_param("~mode", "diff")
    v_req = float(rospy.get_param("~v", 0.4))
    v = min(v_req, V_MAX)
    if v_req > V_MAX:
        rospy.logwarn("请求 v=%.2f 超过硬上限, 已限到 %.2f m/s", v_req, V_MAX)
    R    = float(rospy.get_param("~R", 1.5))
    laps = float(rospy.get_param("~laps", 3))
    d    = 1.0 if float(rospy.get_param("~dir", 1)) >= 0 else -1.0
    ramp = float(rospy.get_param("~ramp", 1.5))
    hz   = float(rospy.get_param("~rate", 50.0))
    topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

    w_full = v / R
    T_lap  = 2.0 * math.pi * R / v
    total_ang = laps * 2.0 * math.pi

    pub = rospy.Publisher(topic, Twist, queue_size=10)
    rate = rospy.Rate(hz)

    def stop():
        t = Twist()
        for _ in range(10):
            pub.publish(t)
            rospy.sleep(0.02)

    rospy.on_shutdown(stop)

    turn = "CCW/左转" if d > 0 else "CW/右转"
    rospy.loginfo("=== circle_run mode=%s v=%.2f R=%.2f laps=%.1f %s ===",
                  mode, v, R, laps, turn)
    rospy.loginfo("初始位姿: 车放场地中心偏一侧 %.2fm, 车头与(指向圆心)垂直, %s",
                  R, turn)
    rospy.loginfo("满速 w=%.3f rad/s, 单圈 %.1fs, 预计总 %.1fs. Ctrl-C 急停.",
                  w_full, T_lap, laps * T_lap)

    rospy.sleep(0.5)  # 等 publisher 连上订阅者

    phi = 0.0   # omni: 车体速度方向累计角
    ang = 0.0   # 已转过的圆心角(判圈用)
    t0 = rospy.Time.now().to_sec()
    last = t0
    while not rospy.is_shutdown() and ang < total_ang:
        now = rospy.Time.now().to_sec()
        dt = now - last
        last = now
        scale = min(1.0, (now - t0) / ramp) if ramp > 1e-3 else 1.0
        v_eff = v * scale
        w_eff = v_eff / R  # 半径恒定: v 与 w 同步渐进

        cmd = Twist()
        if mode == "omni":
            phi += d * w_eff * dt
            cmd.linear.x = v_eff * math.cos(phi)
            cmd.linear.y = v_eff * math.sin(phi)
            cmd.angular.z = 0.0
        else:  # diff
            cmd.linear.x = v_eff
            cmd.angular.z = d * w_eff

        pub.publish(cmd)
        ang += w_eff * dt
        rate.sleep()

    stop()
    rospy.loginfo("done, %.2f laps, stopped.", ang / (2.0 * math.pi))


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
