#!/usr/bin/env python
# -*- coding: utf-8 -*-
# abot_race_nav.py — 比赛一条龙（move_base导航 + 看题/抓取/放置 全自动）
# 前置: bringup + navigate(move_base) + vl_locate + ZachLab grasp 四栈同跑
# 原则: 任何动作失败只logwarn继续——跑完全程+位置准 > 一切
# 启动: rosrun abot_project abot_race_nav.py (或 python abot_race_nav.py)
import rospy, actionlib, subprocess, time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from vl_locate.srv import GetObject

# ---- 站点表(x, y, yaw_rad) ----
PTS = {
    'start':  (0.057, -1.009, -0.012),
    'p1':     (-1.713, -0.680, -0.063),
    'hub4':   (0.103, 0.688, -3.020),
    'p11':    (-2.153, -0.655, -0.070),
    'p2':     (-1.694, 0.514, 0.039),
    'p21':    (-2.096, 0.501, 0.038),
    'p3':     (-1.755, 1.689, 0.078),
    'p31':    (-2.134, 1.661, 0.071),
    'finish': (0.090, 2.320, 1.655),
}
# ---- 任务序: (站点, 动作) 动作: quiz=看题 grasp=抓取 place=放置 None=直接走 ----
ROUTE = [
    ('start', None),
    ('p1', 'quiz'), ('hub4', 'grasp'), ('p11', 'place'),
    ('p2', 'quiz'), ('hub4', 'grasp'), ('p21', 'place'),
    ('p3', 'quiz'), ('hub4', 'grasp'), ('p31', 'place'),
    ('finish', None),
]

NAV_TIMEOUT = 120.0   # 单段导航超时
ACTION_TIMEOUT = 30.0 # 单动作超时
GRASP_WAIT = 18.0     # 抓取臂动作等待

class RaceNav:
    def __init__(self):
        self.fruit = None   # 本轮记忆的目标水果
        self.mb = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('waiting move_base ...')
        self.mb.wait_for_server(rospy.Duration(30))

    # ---------- 导航 ----------
    def goto(self, name):
        x, y, yaw = PTS[name]
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = x
        g.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        g.target_pose.pose.orientation.z = q[2]
        g.target_pose.pose.orientation.w = q[3]
        self.mb.send_goal(g)
        self.mb.wait_for_result(rospy.Duration(NAV_TIMEOUT))
        state = self.mb.get_state()
        ok = (state == 3)  # SUCCEEDED
        rospy.loginfo('[goto %s] %s', name, 'OK' if ok else 'FAIL(state=%d) continue' % state)
        return ok

    # ---------- 动作 ----------
    def do_quiz(self):
        """看题: 拍帧问VLM画面水果, 记忆水果名"""
        try:
            out = subprocess.check_output(
                ['/home/abot/miniconda3/envs/39/bin/python',
                 '/home/abot/catkin_ws/src/abot_project/scripts/quiz_vlm.py', '15'],
                timeout=ACTION_TIMEOUT).strip()
            if out and out != 'FAIL':
                self.fruit = out
                rospy.loginfo('[quiz] 记忆目标水果: %s', out)
                return True
        except Exception as e:
            rospy.logwarn('[quiz] 失败(%s), 继续', e)
        self.fruit = None
        rospy.logwarn('[quiz] 未识别, 本轮跳过抓取')
        return False

    def do_grasp(self):
        """抓取: 用记忆的水果名触发gg.py抓取流程"""
        if not self.fruit:
            rospy.logwarn('[grasp] 无目标水果(看题失败), 跳过')
            return False
        try:
            rospy.set_param('success', False)
            rospy.set_param('/grab', 1)
            rospy.wait_for_service('/vlm_detection', timeout=5)
            vlm = rospy.ServiceProxy('/vlm_detection', GetObject)
            r = vlm.call(self.fruit)
            rospy.loginfo('[grasp] %s: %s', self.fruit,
                          getattr(r, 'message', '?'))
            t0 = time.time()
            while time.time() - t0 < GRASP_WAIT and not rospy.is_shutdown():
                if rospy.get_param('/success', False):
                    rospy.loginfo('[grasp] 成功')
                    return True
                time.sleep(0.5)
            rospy.logwarn('[grasp] 超时(臂未确认), 继续')
            return False
        except Exception as e:
            rospy.logwarn('[grasp] 失败(%s), 继续', e)
            return False

    def do_place(self):
        """放置: 识别框上图像(用本轮水果名)触发gg.py放置"""
        try:
            rospy.set_param('/grab', 0)
            rospy.wait_for_service('/vlm_detection', timeout=5)
            vlm = rospy.ServiceProxy('/vlm_detection', GetObject)
            name = self.fruit if self.fruit else 'box'
            r = vlm.call(name)
            rospy.loginfo('[place] %s: %s', name, getattr(r, 'message', '?'))
            time.sleep(GRASP_WAIT)  # 放置动作等待
            rospy.loginfo('[place] 完成(或超时继续)')
            return True
        except Exception as e:
            rospy.logwarn('[place] 失败(%s), 继续', e)
            return False

    # ---------- 主流程 ----------
    def run(self):
        rospy.loginfo('===== 比赛开始: %d 段 =====', len(ROUTE))
        for i, (st, act) in enumerate(ROUTE):
            rospy.loginfo('--- [%d/%d] -> %s (%s) ---', i+1, len(ROUTE), st, act or 'pass')
            self.goto(st)                       # 失败也继续(原则)
            if act == 'quiz':
                self.do_quiz()
            elif act == 'grasp':
                self.do_grasp()
            elif act == 'place':
                self.do_place()
        rospy.loginfo('===== 全程跑完 =====')

if __name__ == '__main__':
    rospy.init_node('abot_race_nav')
    RaceNav().run()
