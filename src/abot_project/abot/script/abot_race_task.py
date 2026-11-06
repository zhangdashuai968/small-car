#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
abot_race_task.py — 比赛一条龙总控(导航+抓取+放置)
====================================================
用法(前置四栈已起: bringup / navigate / vl_locate / grasp.launch, 车摆0号位):

  rosrun abot abot_race_task.py _fruits:=apple,durian,watermelon

赛前人工把三轮水果的英文名填进 fruits 参数(中英对照):
  苹果=apple  榴莲=durian  西瓜=watermelon  葡萄=grape
  红枣=jujube  山竹=mangosteen  梨=pear

站序(119试跑版坐标): 1号 → 4号[抓fruits[0]] → 1-1[放] → 2号 → 4号[抓fruits[1]]
  → 2-1[放] → 3号 → 4号[抓fruits[2]] → 3-1[放] → 终点
看题站(1/2/3号)只停车路过——名字已赛前填入,不识别。

最高原则: 任何环节失败(导航超时/识别失败/抓取超时)只记日志继续跑,
跑完全程+位置准 >> 单站成功。
"""
import rospy
import math
import actionlib
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from vl_locate.srv import GetObject


class RaceTask(object):
    def __init__(self):
        self.fruits = [s.strip() for s in
                       rospy.get_param('~fruits', 'apple,durian,watermelon').split(',')
                       if s.strip()]
        self.rest_time = float(rospy.get_param('~rest_time', 3))
        self.nav_timeout = float(rospy.get_param('~nav_timeout', 210))
        self.grasp_timeout = float(rospy.get_param('~grasp_timeout', 45))
        self.place_timeout = float(rospy.get_param('~place_timeout', 35))

        # 站点(=119试跑版, 11-06验证8/10那套坐标)
        L = dict()
        L['two']   = Pose(Point(-1.855, -0.631, 0), Quaternion(0, 0, -0.0314, 0.9995))
        L['three'] = Pose(Point(0.082, 0.534, 0), Quaternion(0, 0, -0.9982, 0.0608))
        L['four']  = Pose(Point(-2.260, -0.701, 0), Quaternion(0, 0, 0.1132, 0.9936))
        L['five']  = Pose(Point(-1.840, 0.541, 0), Quaternion(0, 0, 0.0095, 1.000))
        L['six']   = Pose(Point(-2.217, 0.539, 0), Quaternion(0, 0, 0.0095, 1.000))
        L['seven'] = Pose(Point(-1.797, 1.747, 0), Quaternion(0, 0, 0.1495, 0.9888))
        L['eight'] = Pose(Point(-2.186, 1.709, 0), Quaternion(0, 0, -0.0523, 0.9986))
        L['nine']  = Pose(Point(-0.060, 2.320, 0), Quaternion(0, 0, 1.000, 0.000))
        self.locations = L

        # (站点, 动作): None=路过停车 / ('grab',轮次) / ('place',轮次)
        self.plan = [('two', None), ('three', ('grab', 0)), ('four', ('place', 0)),
                     ('five', None), ('three', ('grab', 1)), ('six', ('place', 1)),
                     ('seven', None), ('three', ('grab', 2)), ('eight', ('place', 2)),
                     ('nine', None)]

        self.mb = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('[race] 等待 move_base...')
        if not self.mb.wait_for_server(rospy.Duration(60)):
            rospy.logerr('[race] move_base 60秒未上线,退出——先起导航栈!')
            raise RuntimeError('no move_base')
        rospy.loginfo('[race] 等待 vlm_detection 服务...')
        try:
            rospy.wait_for_service('vlm_detection', timeout=60)
        except rospy.ROSException:
            rospy.logerr('[race] vlm_detection 60秒未上线,退出——先起视觉栈!')
            raise RuntimeError('no vlm service')
        self.vlm = rospy.ServiceProxy('vlm_detection', GetObject)
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf)
        rospy.set_param('/success', False)

        self.result = []   # (站名, 动作, ok)

    # ---------- 导航 ----------
    def goto(self, name):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.locations[name]
        rospy.loginfo('[race] --> %s', name)
        self.mb.send_goal(goal)
        # 快停模式: 位置<15cm且朝向差<23°即提前收工,不等move_base磨完美SUCCEEDED
        t = self.locations[name]
        tx, ty = t.position.x, t.position.y
        qz, qw = t.orientation.z, t.orientation.w
        tgt_yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.mb.wait_for_result(rospy.Duration(0.5)):
                state = self.mb.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo('[race] %s 到站', name)
                    return True
                rospy.logerr('[race] %s 导航失败(state=%d),继续', name, state)
                return False
            try:
                tr = self.tf.lookup_transform('map', 'base_footprint', rospy.Time(0))
                px = tr.transform.translation.x
                py = tr.transform.translation.y
                rz, rw = tr.transform.rotation.z, tr.transform.rotation.w
                yaw = math.atan2(2.0 * rw * rz, 1.0 - 2.0 * rz * rz)
                dist = math.hypot(tx - px, ty - py)
                dyaw = math.atan2(math.sin(yaw - tgt_yaw), math.cos(yaw - tgt_yaw))
                el = (rospy.Time.now() - t0).to_sec()
                if dist < 0.15 and abs(dyaw) < 0.40:
                    self.mb.cancel_goal()
                    rospy.loginfo('[race] %s 到站(快停 %.0fcm/%.0f°,%.0fs)',
                                  name, dist * 100, math.degrees(dyaw), el)
                    return True
                if el > 120.0:
                    self.mb.cancel_goal()
                    near = dist < 0.30
                    rospy.logwarn('[race] %s 快停兜底120s(dist=%.0fcm)按%s计', name,
                                  dist * 100, '到站' if near else '失败')
                    return near
            except Exception:
                pass
        return False

    # ---------- 抓取/放置(厂家 /grab + /success 握手) ----------
    def trigger_arm(self, fruit, grab_mode, timeout):
        mode = '抓取' if grab_mode else '放置'
        rospy.set_param('/grab', bool(grab_mode))
        rospy.set_param('/success', False)
        try:
            resp = self.vlm(fruit)
        except Exception as e:
            rospy.logerr('[race] %s[%s] 识别服务异常:%s,跳过', mode, fruit, str(e))
            return False
        if not resp or not resp.success:
            rospy.logerr('[race] %s[%s] 未识别到,跳过', mode, fruit)
            return False
        rospy.loginfo('[race] %s[%s] 识别OK:%s,等臂完成(%.0fs上限)',
                      mode, fruit, resp.message, timeout)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if rospy.get_param('/success', False):
                rospy.set_param('/success', False)
                rospy.loginfo('[race] %s[%s] 完成(%.1fs)', mode, fruit,
                              (rospy.Time.now() - t0).to_sec())
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr('[race] %s[%s] 等/success超时,放弃继续跑', mode, fruit)
                return False
            rospy.sleep(1.0)
        return False

    # ---------- 主流程 ----------
    def run(self):
        rospy.loginfo('[race] 开赛! 三轮水果: %s', ','.join(self.fruits))
        t_start = rospy.Time.now()
        for i, (station, action) in enumerate(self.plan):
            if rospy.is_shutdown():
                break
            tag = '%d/%d %s' % (i + 1, len(self.plan), station)
            nav_ok = self.goto(station)
            if action is None:
                self.result.append((tag, '路过', nav_ok))
                rospy.sleep(self.rest_time)
                continue
            kind, rnd = action
            fruit = self.fruits[rnd] if rnd < len(self.fruits) else self.fruits[-1]
            if kind == 'grab':
                ok = self.trigger_arm(fruit, True, self.grasp_timeout)
            else:
                ok = self.trigger_arm(fruit, False, self.place_timeout)
            self.result.append((tag, '%s%s' % (kind, fruit), ok))
            rospy.sleep(self.rest_time)
        mins = (rospy.Time.now() - t_start).to_sec() / 60.0
        rospy.loginfo('[race] ===== 全程结束(%.1f分钟) =====', mins)
        for tag, act, ok in self.result:
            rospy.loginfo('[race] %s %-14s %s', tag, act, 'OK' if ok else 'FAIL')
        ok_cnt = sum(1 for _, _, ok in self.result if ok)
        rospy.loginfo('[race] 成功 %d/%d', ok_cnt, len(self.result))


if __name__ == '__main__':
    rospy.init_node('abot_race_task')
    r = RaceTask()
    r.run()
    rospy.loginfo('[race] 任务节点退出')
