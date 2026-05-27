#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
手动导航 + 全局规划（绕开 move_base / TEB / DWA）。

订阅 rviz "2D Nav Goal" (/move_base_simple/goal) -> 在 /map 栅格上跑纯 Python A*
(4 连通, 障碍膨胀避开挡板/围栏) -> 折成 x/y 直线段 -> 复用锁航向 cmd_vel 原语逐段
执行(只沿 xy 轴走, 严格不斜穿) -> 同时发 nav_msgs/Path 给 rviz 显示规划线。

全局规划是静态地图上的确定性算路, 没有局部规划器的反应式避障问题; 执行用轴对齐原语,
符合比赛"只沿 xy 轴、不碰挡板"硬约束。

前置(手动启动)：
  roslaunch abot bringup.launch
  roslaunch abot localize.launch       # map_server 发 /map, amcl 提供 map->base_footprint
然后 rviz 里点 "2D Nav Goal" 发目标即可。

用法：python2 goal_nav.py   (或 rosrun abot goal_nav.py)
中断(Ctrl-C)：立即停车。
"""

import sys
import math
import heapq
import signal

import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


# --- 原语控制器参数 (与 seven_point_test 一致) ---
V_LIN      = 0.20
V_ROT      = 0.40
V_LIN_MIN  = 0.04
V_ROT_MIN  = 0.10
K_LIN      = 4.0
K_LIN_HOLD = 5.0
K_YAW      = 1.5
POS_TOL_GOAL = 0.04   # 终点位置容差
POS_TOL_WP   = 0.10   # 中间拐点容差(松一点, 不在每个拐点急停)
YAW_TOL    = 0.03
RATE_HZ    = 20
STEP_TIMEOUT = 30.0

MAP_FRAME  = 'map'
BASE_FRAME = 'base_footprint'

OBST_THRESH = 50      # OccupancyGrid >=此值视为障碍; -1(未知)也视为障碍


def _norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class GoalNav(object):
    def __init__(self):
        rospy.init_node('goal_nav')
        self.inflation_m = rospy.get_param('~inflation_radius', 0.20)

        self._tf = tf.TransformListener()
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._path_pub = rospy.Publisher('~path', Path, queue_size=1, latch=True)

        # 地图(膨胀后阻挡位图)
        self.w = self.h = 0
        self.res = 0.05
        self.ox = self.oy = 0.0
        self.block = None
        self._goal = None
        self._busy = False

        rospy.Subscriber('map', OccupancyGrid, self._map_cb, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._goal_cb, queue_size=1)

        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigint)
        rospy.loginfo('goal_nav ready, 在 rviz 用 2D Nav Goal 发目标')

    # ---------- 地图 ----------
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
        rospy.loginfo('map %dx%d res=%.3f inflate=%.2fm(%dcell) loaded', w, h, self.res, self.inflation_m, R)

    def _w2g(self, wx, wy):
        return int((wx - self.ox) / self.res), int((wy - self.oy) / self.res)

    def _g2w(self, gx, gy):
        return self.ox + (gx + 0.5) * self.res, self.oy + (gy + 0.5) * self.res

    def _blocked(self, gx, gy):
        if gx < 0 or gy < 0 or gx >= self.w or gy >= self.h:
            return True
        return self.block[gy * self.w + gx] == 1

    def _nearest_free(self, gx, gy, max_r=12):
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
                # 重建 cell 路径
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

    # ---------- 位姿/原语 ----------
    def _cur(self):
        try:
            trans, rot = self._tf.lookupTransform(MAP_FRAME, BASE_FRAME, rospy.Time(0))
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

    def _drive_to_point(self, tx, ty, hold_yaw, pos_tol, timeout=STEP_TIMEOUT):
        rate = rospy.Rate(RATE_HZ)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            cur = self._cur()
            if cur is None:
                self._stop(); return False
            ex = tx - cur[0]
            ey = ty - cur[1]
            if math.hypot(ex, ey) < pos_tol:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn('    drive_to (%.2f,%.2f) TIMEOUT', tx, ty)
                self._stop(); return False
            if abs(ex) < abs(ey):
                vx = _clamp(K_LIN_HOLD * ex, -V_LIN, V_LIN)
                vy = _clamp(K_LIN * ey, -V_LIN, V_LIN)
            else:
                vx = _clamp(K_LIN * ex, -V_LIN, V_LIN)
                vy = _clamp(K_LIN_HOLD * ey, -V_LIN, V_LIN)
            sp = math.hypot(vx, vy)
            if 1e-6 < sp < V_LIN_MIN:
                vx *= V_LIN_MIN / sp
                vy *= V_LIN_MIN / sp
            th = cur[2]
            c = math.cos(th); s = math.sin(th)
            tw = Twist()
            tw.linear.x =  c * vx + s * vy
            tw.linear.y = -s * vx + c * vy
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
                rospy.logwarn('    rotate_to %.2f TIMEOUT', target_yaw)
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

    # ---------- 规划 + 执行 ----------
    def _handle_goal(self, goal_msg):
        if self.block is None:
            rospy.logwarn('no /map yet, ignore goal'); return
        cur = self._cur()
        if cur is None:
            rospy.logwarn('no map->base tf, 确认 amcl 收敛'); return

        gx_goal = goal_msg.pose.position.x
        gy_goal = goal_msg.pose.position.y
        q = goal_msg.pose.orientation
        goal_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        scell = self._w2g(cur[0], cur[1])
        gcell = self._w2g(gx_goal, gy_goal)
        scell = self._nearest_free(*scell) or scell   # 起点落在膨胀里时就近放行
        gcell = self._nearest_free(*gcell)
        if gcell is None:
            rospy.logwarn('goal 在障碍/膨胀区内且附近无空闲, 放弃'); return

        cells = self._astar(scell, gcell)
        if cells is None:
            rospy.logwarn('A* 找不到到 (%.2f,%.2f) 的路径', gx_goal, gy_goal); return

        corners = self._simplify(cells)
        corners_w = [self._g2w(gx, gy) for gx, gy in corners]
        corners_w[-1] = (gx_goal, gy_goal)   # 末点用精确目标坐标
        self._publish_path(corners_w)
        rospy.loginfo('规划成功: %d cell -> %d 直线段, 开始执行', len(cells), len(corners_w) - 1)

        hold = cur[2]
        for i, (wx, wy) in enumerate(corners_w):
            tol = POS_TOL_GOAL if i == len(corners_w) - 1 else POS_TOL_WP
            rospy.loginfo('  leg%d -> (%.3f, %.3f)', i + 1, wx, wy)
            if not self._drive_to_point(wx, wy, hold, tol):
                rospy.logerr('  leg%d 失败, 中止', i + 1)
                self._stop(); return
        rospy.loginfo('  rotate -> %.2f', goal_yaw)
        self._rotate_to(goal_yaw)
        rospy.loginfo('到达目标')

    def _goal_cb(self, msg):
        self._goal = msg

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self._goal is not None and not self._busy:
                self._busy = True
                g = self._goal
                self._goal = None
                try:
                    self._handle_goal(g)
                finally:
                    self._busy = False
            rate.sleep()


if __name__ == '__main__':
    GoalNav().run()
