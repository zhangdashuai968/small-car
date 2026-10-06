#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from riki_msgs.msg import Servo
from vl_locate.srv import GetObject
import tf.transformations as tf


class AutoNavigationGrasp:
    def __init__(self):
        rospy.init_node('auto_navigation_grasp')

        # Move base client
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()

        # Publishers
        self.servo_pub = rospy.Publisher('servo', Servo, queue_size=1)

        # Service client
        rospy.wait_for_service('/vlm_detection')
        self.vlm_client = rospy.ServiceProxy('/vlm_detection', GetObject)

        # === 改动说明（7点→12点）===
        # 旧版: 7个点位，用3个独立列表(waypoints/actions/objects)分别存储
        #       旧坐标: (3.6,0.6) (2.4,0.6) ... 等7个点，动作只有 True(抓)/False(放)/None(结束)
        #       旧目标物体: 蓝方块、红方块、绿方块
        # 新版: 12个点位，合并为1个字典列表(tasks)，每个点位包含名称/坐标/动作/目标
        #       新坐标: 键盘遥控重新采集(map坐标系)，每轮抓取增加"路过"点
        #       新目标物体: 苹果、榴莲、西瓜
        #       新增动作类型: pass(路过不停留)、finish(终点回舵机)
        # ============================

        # 12点任务: 3轮抓取（苹果→榴莲→西瓜），增加两个跨区途径点
        self.tasks = [
            # 第1轮: 苹果
            {'name': 'point_01_pass', 'pose': (-1.616, 0.065, -1.600), 'action': 'pass', 'object': None},
            {'name': 'point_02_grab', 'pose': (-2.712, -0.185, -1.600), 'action': 'grab', 'object': 'yellow'},
            {'name': 'point_03_place', 'pose': (-1.616, 0.065, -1.600), 'action': 'place', 'object': '图片中心点'},
            {'name': 'point_03_04_exit_pass', 'pose': (-0.617, 0.987, -1.600), 'action': 'pass', 'object': None},
            # 第2轮: 榴莲
            {'name': 'point_04_pass', 'pose': (-1.819, 1.288, -1.600), 'action': 'pass', 'object': None},
            {'name': 'point_05_grab', 'pose': (-2.994, 0.888, -1.600), 'action': 'grab', 'object': 'red'},
            {'name': 'point_06_place', 'pose': (-1.819, 1.288, -1.600), 'action': 'place', 'object': '图片中心点'},
            {'name': 'point_06_07_exit_pass', 'pose': (-0.921, 2.379, -1.600), 'action': 'pass', 'object': None},
            # 第3轮: 西瓜
            {'name': 'point_07_pass', 'pose': (-2.186, 2.469, -1.600), 'action': 'pass', 'object': None},
            {'name': 'point_08_grab', 'pose': (-3.306, 2.010, -1.600), 'action': 'grab', 'object': 'green'},
            {'name': 'point_09_place', 'pose': (-2.186, 2.469, -1.600), 'action': 'place', 'object': '图片中心点'},
            # 终点: 舵机归位
            {'name': 'point_10_finish', 'pose': (-0.873, 3.447, -1.600), 'action': 'finish', 'object': None},
        ]

    def send_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        q = tf.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)

        self.move_base.send_goal(goal)
        return self.move_base.wait_for_result()

    def set_servo(self):
        servo_msg = Servo()
        servo_msg.Servo1 = 88
        servo_msg.Servo2 = 20
        self.servo_pub.publish(servo_msg)

    def set_grab_param(self, grab):
        rospy.set_param('/grab', grab)

    def call_vlm_detection(self, object_name):
        try:
            response = self.vlm_client(object_name)
            return response
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def run_task_action(self, task):
        """根据动作类型执行对应操作（新增方法，旧版在run()中内联处理）"""
        action = task['action']
        if action == 'pass':       # 路过: 到达后直接跳过，不触发任何动作
            rospy.loginfo("Pass point: %s", task['name'])
            return
        if action == 'finish':     # 终点: 舵机归位，任务结束（新增）
            self.set_grab_param(0)
            self.set_servo()
            rospy.loginfo("Final point reached - task complete")
            return

        # grab/place: 设置抓取参数 → 调用VLM检测 → 等待机械臂完成
        grab = action == 'grab'
        self.set_grab_param(1 if grab else 0)
        rospy.loginfo("%s: %s", "Grabbing" if grab else "Placing", task['object'])
        self.call_vlm_detection(task['object'])
        time.sleep(5)  # 等待5秒让机械臂完成抓/放动作

    def run(self):
        # 初始化: 舵机回初始位置(Servo1=88, Servo2=20)，/grab参数清零
        self.set_servo()
        self.set_grab_param(0)
        time.sleep(5)  # 等待初始化完成

        # 遍历12个点位，逐个导航并执行动作
        for i, task in enumerate(self.tasks):
            x, y, yaw = task['pose']

            rospy.loginfo("Navigating to point %d/%d %s: (%.2f, %.2f, %.2f)",
                          i + 1, len(self.tasks), task['name'], x, y, yaw)

            if self.send_goal(x, y, yaw):   # 阻塞等待导航完成
                rospy.loginfo("Reached point %d: %s", i + 1, task['name'])
                self.run_task_action(task)   # 到达后执行对应动作
            else:
                rospy.logwarn("Failed to reach point %d: %s", i + 1, task['name'])

        rospy.loginfo("12-point auto navigation and grasp sequence completed")


if __name__ == '__main__':
    try:
        auto_nav = AutoNavigationGrasp()
        auto_nav.run()
    except rospy.ROSInterruptException:
        pass0
