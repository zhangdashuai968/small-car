#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
多点位顺序巡逻 + A* 全局避障。
直接发 cmd_vel 原语控制，不依赖 move_base/TEB/DWA。

每个目标点位通过 A* 全局规划（静态地图 + 障碍膨胀）生成无障碍路径，
简化为 x/y 直线拐点序列后逐段执行，全程保持车头朝向不漂。

位姿来源 tf map->base_footprint(世界坐标系)，由 amcl+EKF 提供。
地图来源 /map topic (map_server)，由 localize.launch 加载。

前置(手动启动)：
  roscore
  roslaunch abot bringup.launch
  roslaunch abot localize.launch       # amcl + map_server + lidar（无 move_base）

用法：python2 auto_task_runner.py [waypoints.yaml]
中断(Ctrl-C)/异常或刷新：停止后回舵机 home，/grab=False
"""

import os
import sys
import math
import heapq
import signal
import time

import rospy
import yaml
import tf

from geometry_msgs.msg import Twist, PoseStamped
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import OccupancyGrid, Path
from riki_msgs.msg import Servo
from vl_locate.srv import GetObject


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
POS_TOL_WP = 0.10  # m     中间拐点容差(松一点，不在每个拐点急停)
YAW_TOL   = 0.03   # rad   终点角度容差(~1.7°)
RATE_HZ   = 20
STEP_TIMEOUT = 30.0  # s  单步超时(fail-fast)

# --- A* 规划参数 ---
OBST_THRESH = 50       # OccupancyGrid >=此值视为障碍; -1(未知)也视为障碍
INFLATION_R = 0.30     # m 障碍膨胀半径 (0.20→0.30, 避免擦边撞墙)

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
        self.inflation_m = rospy.get_param('~inflation_radius', INFLATION_R)

        self._tf = tf.TransformListener()
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._mb_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self._path_pub = rospy.Publisher('~path', Path, queue_size=1, latch=True)
        self.servo_pub = rospy.Publisher('servo', Servo, queue_size=1, latch=True)
        self.vlm = None  # 延迟加载，避免依赖完整抓取栈

        # --- 地图(膨胀后阻挡位图) ---
        self.w = self.h = 0
        self.res = 0.05
        self.ox = self.oy = 0.0
        self.block = None
        rospy.Subscriber('map', OccupancyGrid, self._map_cb, queue_size=1)

        rospy.loginfo('waiting tf %s->%s ...', MAP_FRAME, BASE_FRAME)
        try:
            self._tf.waitForTransform(MAP_FRAME, BASE_FRAME, rospy.Time(0), rospy.Duration(10.0))
        except tf.Exception:
            rospy.logwarn('no %s->%s tf, 确认 localize.launch 已启动 amcl 已收敛', MAP_FRAME, BASE_FRAME)

        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigint)
        time.sleep(0.5)
        self.refresh()

    # ---------- 地图处理 ----------
    def _map_cb(self, msg):
        self.w = msg.info.width
        self.h = msg.info.height
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        data = msg.data
        w, h = self.w, self.h
        block = bytearray(w * h)
        R = int(math.ceil(self.inflation_m / self.res))
        # 膨胀核
        offs = [(dx, dy) for dx in range(-R, R + 1) for dy in range(-R, R + 1)
                if dx * dx + dy * dy <= R * R]
        for idx in range(w * h):
            v = data[idx]
            if v < 0 or v >= OBST_THRESH:
                gx = idx % w
                gy = idx // w
                for dx, dy in offs:
                    nx = gx + dx
                    ny = gy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        block[ny * w + nx] = 1
        self.block = block
        rospy.loginfo('map %dx%d res=%.3f inflate=%.2fm(%dcell) loaded',
                      w, h, self.res, self.inflation_m, R)

    def _w2g(self, wx, wy):
        return int((wx - self.ox) / self.res), int((wy - self.oy) / self.res)

    def _g2w(self, gx, gy):
        return self.ox + (gx + 0.5) * self.res, self.oy + (gy + 0.5) * self.res

    def _blocked(self, gx, gy):
        if gx < 0 or gy < 0 or gx >= self.w or gy >= self.h:
            return True
        return self.block[gy * self.w + gx] == 1

    def _nearest_free(self, gx, gy, max_r=12):
        """找最近空闲栅格，避免起点/终点落在膨胀区内"""
        if not self._blocked(gx, gy):
            return (gx, gy)
        for r in range(1, max_r + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    if not self._blocked(gx + dx, gy + dy):
                        return (gx + dx, gy + dy)
        return None

    # ---------- A* (4 连通) ----------
    def _astar(self, start, goal):
        w = self.w
        sidx = start[1] * w + start[0]
        gidx = goal[1] * w + goal[0]
        gx_g, gy_g = goal
        openh = [(0, sidx)]
        gscore = {sidx: 0}
        came = {}
        while openh:
            _, cur = heapq.heappop(openh)
            if cur == gidx:
                cells = [cur]
                while cur in came:
                    cur = came[cur]
                    cells.append(cur)
                cells.reverse()
                return [(c % w, c // w) for c in cells]
            cgc = gscore[cur]
            cx = cur % w
            cy = cur // w
            for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                if self._blocked(nx, ny):
                    continue
                nidx = ny * w + nx
                ng = cgc + 1
                if ng < gscore.get(nidx, 1 << 30):
                    gscore[nidx] = ng
                    came[nidx] = cur
                    f = ng + abs(nx - gx_g) + abs(ny - gy_g)
                    heapq.heappush(openh, (f, nidx))
        return None

    @staticmethod
    def _simplify(cells):
        """只保留方向变化的拐点 -> x/y 直线段"""
        if len(cells) <= 2:
            return cells
        out = [cells[0]]
        for i in range(1, len(cells) - 1):
            dx0 = cells[i][0] - cells[i - 1][0]
            dy0 = cells[i][1] - cells[i - 1][1]
            dx1 = cells[i + 1][0] - cells[i][0]
            dy1 = cells[i + 1][1] - cells[i][1]
            if (dx0, dy0) != (dx1, dy1):
                out.append(cells[i])
        out.append(cells[-1])
        return out

    def _publish_path(self, corners_w):
        path = Path()
        path.header.frame_id = MAP_FRAME
        path.header.stamp = rospy.Time.now()
        for wx, wy in corners_w:
            ps = PoseStamped()
            ps.header.frame_id = MAP_FRAME
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)

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
        """闭环直行到 map 下 (tx,ty)，全程锁 hold_yaw，位移到达时无需笔直直线。"""
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
            if abs(ex) < abs(ey):
                vx_map = _clamp(K_LIN_HOLD * ex, -V_LIN, V_LIN)
                vy_map = _clamp(K_LIN * ey, -V_LIN, V_LIN)
            else:
                vx_map = _clamp(K_LIN * ex, -V_LIN, V_LIN)
                vy_map = _clamp(K_LIN_HOLD * ey, -V_LIN, V_LIN)
            sp = math.hypot(vx_map, vy_map)
            if 1e-6 < sp < V_LIN_MIN:
                vx_map *= V_LIN_MIN / sp
                vy_map *= V_LIN_MIN / sp
            th = cur[2]
            c = math.cos(th); s = math.sin(th)
            tw = Twist()
            tw.linear.x =  c * vx_map + s * vy_map
            tw.linear.y = -s * vx_map + c * vy_map
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
        """移动到 (x, y, yaw)。
        如果 /map 已加载：A* 全局规划 → 拐点序列 → 逐段执行（有避障）。
        否则：回退到原两段 L 形盲走。
        """
        cur = self._cur()
        if cur is None:
            rospy.logerr('no tf pose, abort goto')
            return False
        hold = cur[2]

        # --- 尝试 A* 规划 ---
        if self.block is not None:
            scell = self._w2g(cur[0], cur[1])
            gcell = self._w2g(x, y)
            scell = self._nearest_free(*scell) or scell
            gcell = self._nearest_free(*gcell)
            if gcell is None:
                rospy.logwarn('目标 (%.2f,%.2f) 在障碍区内且附近无空闲, 回退盲走', x, y)
            else:
                cells = self._astar(scell, gcell)
                if cells is None:
                    rospy.logwarn('A* 找不到到 (%.2f,%.2f) 的路径, 回退盲走', x, y)
                else:
                    corners = self._simplify(cells)
                    corners_w = [self._g2w(gx, gy) for gx, gy in corners]
                    corners_w[-1] = (x, y)  # 末点用精确目标坐标
                    self._publish_path(corners_w)
                    rospy.loginfo('A* 规划: %d cell -> %d 直线段', len(cells), len(corners_w) - 1)

                    for i, (wx, wy) in enumerate(corners_w):
                        tol = POS_TOL if i == len(corners_w) - 1 else POS_TOL_WP
                        rospy.loginfo('  leg%d -> (%.3f, %.3f) tol=%.2f', i + 1, wx, wy, tol)
                        if not self._drive_to_point(wx, wy, hold, tol):
                            rospy.logerr('  leg%d 失败', i + 1)
                            return False
                    rospy.loginfo('  旋转 -> %.2f', yaw)
                    return self._rotate_to(yaw)

        # --- 回退: 原 L 形盲走 ---
        rospy.loginfo('  盲走模式 (无 /map 或 A* 失败) -> (%.2f, %.2f, %.2f)', x, y, yaw)
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

    # ---------- 抓取/放置 ----------
    def do_action(self, action, obj):
        """执行点位动作: grab=抓取, place=放置, pass=跳过"""
        if action == 'pass':
            return True

        # 1. 舵机归位（每次动作前确保机械臂在 home 位）
        rospy.loginfo('  servo home → %s', SERVO_HOME)
        self._servo_home()
        rospy.sleep(1.0)

        # 2. 设置抓取/放置参数
        is_grab = (action == 'grab')
        rospy.set_param('/grab', 1 if is_grab else 0)
        rospy.loginfo('  /grab = %d (%s)', 1 if is_grab else 0, action)

        # 3. VLM 检测（延迟连接，容忍服务不可用）
        if self.vlm is None:
            try:
                rospy.wait_for_service('/vlm_detection', timeout=5.0)
                self.vlm = rospy.ServiceProxy('/vlm_detection', GetObject)
                rospy.loginfo('  VLM service connected')
            except rospy.ROSException:
                rospy.logwarn('  VLM service not available (vl_locate not running?), 盲抓')
                self.vlm = False

        if self.vlm and self.vlm is not False:
            try:
                resp = self.vlm(obj)
                rospy.loginfo('  VLM detected: %s', resp)
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logwarn('  vlm call failed: %s, 继续盲抓', e)
        else:
            rospy.logwarn('  跳过 VLM, 直接%s', '抓取' if is_grab else '放置')

        # 4. 等待机械臂执行
        rospy.sleep(GRASP_WAIT_SEC)
        return True

    def run(self):
        # 等待地图加载，避免第一个航点盲走
        if self.block is None:
            rospy.loginfo('waiting /map ...')
            timeout = rospy.Time.now() + rospy.Duration(10.0)
            while self.block is None and not rospy.is_shutdown():
                if rospy.Time.now() > timeout:
                    rospy.logwarn('/map 超时(10s)，将使用盲走模式')
                    break
                rospy.sleep(0.2)

        for i, wp in enumerate(self.waypoints):
            name = wp.get('name', 'wp%d' % i)
            action = wp.get('action', 'pass')
            rospy.loginfo('[%d/%d] %s -> (%.3f, %.3f, %.2f) action=%s',
                          i + 1, len(self.waypoints), name,
                          wp['x'], wp['y'], wp['yaw'], action)
            if not self.goto(wp['x'], wp['y'], wp['yaw'], wp.get('frame', 'map')):
                rospy.logerr('goto %s failed, skip', name)
                continue
            if not self.do_action(action, wp.get('object', '')):
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
