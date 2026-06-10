#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Goal 抢占零速注入器。
监听 /move_base/goal，当新 goal 在 preempt_interval 秒内到达时，
立即向 /cmd_vel 发布零速 Twist。
解决 Goal 抢占时 move_base 不发送零速度、机器人靠残留动量滑行的问题。

用法:
    python2 goal_preemption_brake.py
    python2 goal_preemption_brake.py --preempt-interval 10.0
"""
import sys
import rospy
from geometry_msgs.msg import Twist          # /cmd_vel 的消息类型，控制底盘线速度+角速度
from move_base_msgs.msg import MoveBaseActionGoal  # /move_base/goal 话题的消息类型


class GoalPreemptionBrake(object):
    """
    监听 move_base 的 goal 话题。
    如果上一个 goal 还在执行中就被新 goal 抢占了（两次 goal 间隔 < 阈值），
    立即向底盘注入一帧零速度，把车刹住，避免靠惯性滑行。
    """

    def __init__(self, preempt_interval=10.0):
        """
        preempt_interval: 判定为"抢占"的时间阈值（秒）。
                          如果新 goal 距离上一次 goal 不到这个时间，就认为是抢占。
        """
        # 初始化 ROS 节点，节点名 goal_preemption_brake
        rospy.init_node('goal_preemption_brake')

        self.preempt_interval = preempt_interval

        # 记录上一次收到 goal 的时间戳。初始化为 epoch=0，表示"还没收到过 goal"
        self.last_goal_time = rospy.Time(0)

        # ---- 订阅 /move_base/goal ----
        # move_base 是 ROS 的导航框架，它对外暴露一个 action 接口。
        # /move_base/goal 是 action 的内部话题，由 action client 发给 server。
        # 只要有人调 move_base 的 send_goal，这条话题就会收到一条消息。
        # 我们不需要解析 goal 内容，只需要知道"新 goal 来了"的时间点。
        self.goal_sub = rospy.Subscriber(
            '/move_base/goal', MoveBaseActionGoal, self._goal_cb)

        # ---- 发布 /cmd_vel ----
        # /cmd_vel 是底盘控制话题，Twist 消息里的所有字段默认都是 0。
        # 直接 publish(Twist()) 等价于让底盘急停。
        # queue_size=1：只保留最新一条，旧的就丢掉（急停不需要排队）。
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.loginfo('goal_preemption_brake ready, '
                      'preempt_interval=%.1fs', self.preempt_interval)

    def _goal_cb(self, msg):
        """
        回调函数：每次 /move_base/goal 上有新消息时自动触发。
        不关心 msg 的具体内容，只关心"新 goal 到达"这个事件的时间。

        逻辑：
            ① 计算当前时间与上一次 goal 到达时间的差值 dt
            ② 如果 dt < preempt_interval（说明上一次 goal 还没执行完就被抢占了）
               → 立即向 /cmd_vel 发一个全零 Twist，让底盘急停
            ③ 无论是否注入零速，都更新 last_goal_time
        """
        now = rospy.Time.now()                        # 当前 ROS 时间
        dt = (now - self.last_goal_time).to_sec()     # 距离上次 goal 的秒数

        # 条件一：last_goal_time > 0  →  确保已经收到过至少一个 goal
        #          （初始化时 last_goal_time = 0，第一条 goal 不能算"抢占"）
        # 条件二：dt < preempt_interval →  两次 goal 间隔低于阈值，判定为抢占
        if self.last_goal_time.to_sec() > 0 and dt < self.preempt_interval:
            rospy.logwarn('Goal preempted! dt=%.2fs < %.1fs -> '
                          'injecting zero velocity', dt, self.preempt_interval)
            # Twist() 构造时所有字段默认为 0：
            #   linear.x=0, linear.y=0, linear.z=0   → 线速度为零
            #   angular.x=0, angular.y=0, angular.z=0 → 角速度为零
            # 全零 = 底盘急停，刹住惯性滑行
            self.cmd_pub.publish(Twist())

        # 更新"上一次 goal 时间"，供下一次回调比较
        self.last_goal_time = now

    def run(self):
        """
        进入 ROS 事件循环，阻塞等待回调触发。
        等价于 while not rospy.is_shutdown(): sleep(0.1)，但由 ROS 调度。
        """
        rospy.spin()


if __name__ == '__main__':
    # ---- 命令行参数解析 ----
    # 默认抢占间隔 10 秒，可通过 --preempt-interval 覆盖
    interval = 10.0
    args = sys.argv[1:]
    if '--preempt-interval' in args:
        try:
            idx = args.index('--preempt-interval')
            interval = float(args[idx + 1])
        except (ValueError, IndexError):
            rospy.logerr('invalid --preempt-interval value')
            sys.exit(1)

    # 实例化并启动节点
    GoalPreemptionBrake(preempt_interval=interval).run()
