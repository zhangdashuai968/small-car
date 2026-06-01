#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ABOT 导航 bag 分析脚本 — 三段计时 + 轨迹/路径/TEB 可视化
三段计时逻辑:
  准备   = goto() 入口 → 第一条非零 cmd_vel
  快速路 = body 系合速度 >= 0.16 m/s (80% V_LIN)
  微调   = 合速度 < 0.16 m/s 减速段 + 原地旋转
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import csv, os, json
from collections import defaultdict

# ============== 配置 ==============
CSV_DIR = r"E:\abot_study\bags\csv"
OUT_DIR = r"E:\abot_study\bags"
V_LIN = 0.2
V_THRESHOLD = 0.16

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Noto Sans CJK SC']
plt.rcParams['axes.unicode_minus'] = False

# ============== 数据加载 ==============
def load_csv(filename):
    path = os.path.join(CSV_DIR, filename)
    rows = []
    with open(path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows

def parse_time_ns(val):
    try:
        return float(val) / 1e9
    except (ValueError, TypeError):
        return None

print("=" * 60)
print("[LOAD] 加载 bag 数据...")

odom_raw = load_csv("odom.csv")
cmd_raw  = load_csv("cmd_vel.csv")
goals_raw = load_csv("goals.csv")
results_raw = load_csv("results.csv")
global_plan_raw = load_csv("global_plan.csv")
teb_local_raw = load_csv("teb_local.csv")
teb_poses_raw = load_csv("teb_poses.csv")

# odom
odom_data = []
for r in odom_raw:
    t = parse_time_ns(r['%time'])
    if t is None: continue
    odom_data.append({
        't': t,
        'x': float(r.get('field.pose.pose.position.x', 0)),
        'y': float(r.get('field.pose.pose.position.y', 0)),
        'vx': float(r.get('field.twist.twist.linear.x', 0)),
        'vy': float(r.get('field.twist.twist.linear.y', 0)),
    })

# cmd_vel
cmd_data = []
for r in cmd_raw:
    t = parse_time_ns(r['%time'])
    if t is None: continue
    cmd_data.append({
        't': t,
        'lx': float(r['field.linear.x']),
        'ly': float(r['field.linear.y']),
        'az': float(r['field.angular.z']),
    })

# goals
goals = []
for r in goals_raw:
    t = parse_time_ns(r['%time'])
    if t is None: continue
    goals.append({
        't': t,
        'x': float(r['field.goal.target_pose.pose.position.x']),
        'y': float(r['field.goal.target_pose.pose.position.y']),
    })
    print(f"  [GOAL] Goal {len(goals)}: t={t:.3f}  ({goals[-1]['x']:.3f}, {goals[-1]['y']:.3f})")

# global_plan: aggregate poses per msg timestamp
def aggregate_pose_msgs(raw, prefix='field.poses'):
    """将 rostopic echo -p 输出的多行 pose 消息聚合为 {t, poses: [(x,y),...]}"""
    result = []
    current_poses = []
    current_t = None
    last_time = None
    for r in raw:
        t = parse_time_ns(r['%time'])
        if t is None: continue
        px = float(r.get(f'{prefix}.pose.position.x', 0))
        py = float(r.get(f'{prefix}.pose.position.y', 0))
        if last_time is not None and abs(t - last_time) > 1e-9:
            if current_poses and current_t is not None:
                result.append({'t': current_t, 'poses': current_poses})
            current_poses = []
            current_t = t
        current_poses.append((px, py))
        last_time = t
    if current_poses and current_t is not None:
        result.append({'t': current_t, 'poses': current_poses})
    return result

global_paths = aggregate_pose_msgs(global_plan_raw)
teb_locals = aggregate_pose_msgs(teb_local_raw)
teb_poses_seq = aggregate_pose_msgs(teb_poses_raw)

bag_start = odom_data[0]['t']
print(f"\n  Odom: {len(odom_data)}  Cmd_vel: {len(cmd_data)}  Global plan: {len(global_paths)}  TEB local: {len(teb_locals)}  TEB poses: {len(teb_poses_seq)}")

# ============== 辅助函数 ==============
def find_first_nonzero_cmd(cmd_data, after_t, until_t=None):
    """找 after_t 之后第一条非零 cmd_vel"""
    for c in cmd_data:
        if c['t'] < after_t:
            continue
        if until_t and c['t'] > until_t:
            break
        if abs(c['lx']) > 0.001 or abs(c['ly']) > 0.001 or abs(c['az']) > 0.001:
            return c['t']
    return after_t  # fallback

def find_odom_at(odom_data, target_t):
    """找最接近 target_t 的 odom"""
    best = None
    best_dist = float('inf')
    for o in odom_data:
        d = abs(o['t'] - target_t)
        if d < best_dist:
            best_dist = d
            best = o
    return best

def compute_trajectory_dist(odom_data, t_start, t_end):
    """计算 odom 轨迹在时间段内的累计里程"""
    total = 0.0
    prev = None
    for o in odom_data:
        if o['t'] < t_start: continue
        if o['t'] > t_end: break
        if prev is not None:
            total += np.sqrt((o['x'] - prev['x'])**2 + (o['y'] - prev['y'])**2)
        prev = o
    return total

# ============== 三段计时分析 (考虑 goal 抢占) ==============
print("\n" + "=" * 60)
print("[TIME]  三段计时分析 (Goal 间抢占处理)\n")

results = []
for i, goal in enumerate(goals):
    goal_t = goal['t']
    gx, gy = goal['x'], goal['y']

    # 该 goal 的终止时间: 下一个 goal 的时间 或 bag 结束
    if i + 1 < len(goals):
        next_goal_t = goals[i + 1]['t']
    else:
        next_goal_t = odom_data[-1]['t']  # 最后一个 goal 用到 bag 结束

    # 起点: goal 时刻的 odom 位置
    start_odom = find_odom_at(odom_data, goal_t)
    if start_odom is None:
        continue
    start_x, start_y = start_odom['x'], start_odom['y']

    # Phase 1: 准备 (goal → 首次非零 cmd_vel, 且不超过下一个 goal)
    phase1_end = find_first_nonzero_cmd(cmd_data, goal_t, next_goal_t)
    t_prepare = phase1_end - goal_t if phase1_end > goal_t else 0

    # Phase 2: 快速路 (>= V_THRESHOLD, 不超过下一个 goal)
    phase2_start = phase1_end
    last_fast_t = phase2_start
    for c in cmd_data:
        if c['t'] < phase2_start: continue
        if c['t'] > next_goal_t: break
        v = np.sqrt(c['lx']**2 + c['ly']**2)
        if v >= V_THRESHOLD:
            last_fast_t = c['t']

    phase2_end = last_fast_t
    t_fast = phase2_end - phase2_start if phase2_end > phase2_start else 0

    # Phase 3: 微调 (phase2_end → 到达 goal 或 被下一 goal 抢占)
    phase3_start = phase2_end
    phase3_end = phase3_start

    # 找到达 goal 的时刻
    arrived_t = None
    for o in odom_data:
        if o['t'] < phase3_start: continue
        if o['t'] > next_goal_t + 1.0: break  # 允许稍超
        dist = np.sqrt((o['x'] - gx)**2 + (o['y'] - gy)**2)
        if dist < 0.15:
            arrived_t = o['t']
            break

    if arrived_t is not None:
        phase3_end = arrived_t
    else:
        # 没到达 → 取 next_goal_t 或最后一条 odom
        phase3_end = next_goal_t

    t_fine = max(0, phase3_end - phase3_start)

    total_t = phase3_end - goal_t
    fast_dist = compute_trajectory_dist(odom_data, phase2_start, phase2_end)
    straight_dist = np.sqrt((gx - start_x)**2 + (gy - start_y)**2)

    # Goal 是否被抢占
    preempted = (i + 1 < len(goals)) and (arrived_t is None or arrived_t > next_goal_t)

    result = {
        'goal_idx': i + 1,
        'goal_t': goal_t,
        'goal': (gx, gy),
        'start': (start_x, start_y),
        'straight_dist': straight_dist,
        't_prepare': t_prepare,
        't_fast': t_fast,
        't_fine': t_fine,
        't_total': total_t,
        'fast_dist': fast_dist,
        'phase2_start': phase2_start,
        'phase2_end': phase2_end,
        'phase3_start': phase3_start,
        'phase3_end': phase3_end,
        'preempted': preempted,
        'next_goal_t': next_goal_t,
    }
    results.append(result)

    flag = ' [!!] 被抢占' if preempted else ''
    print(f"[GOAL] Goal {i+1}: ({start_x:.2f},{start_y:.2f}) → ({gx:.2f},{gy:.2f})  直线 {straight_dist:.2f}m{flag}")
    print(f"   ├ 准备   {t_prepare:.2f}s")
    if t_fast > 0:
        print(f"   ├ 快速路 {t_fast:.2f}s  (≥0.16m/s, {fast_dist:.2f}m, 均速 {fast_dist/t_fast:.2f}m/s)")
    else:
        print(f"   ├ 快速路 {t_fast:.2f}s  (无高速段)")
    print(f"   ├ 微调   {t_fine:.2f}s")
    print(f"   └ 总计   {total_t:.2f}s")
    print()

# ============== 可视化 ==============
print("=" * 60)
print("[PLOT] 生成分析图表...")

C_ACTUAL  = '#58a6ff'
C_GLOBAL  = '#f85149'
C_TEB     = '#3fb950'
C_FAST    = '#d2991d'
C_FINE    = '#a371f7'
C_PREPARE = '#db61a2'
C_GOAL    = '#f85149'
C_START   = '#3fb950'

fig = plt.figure(figsize=(22, 20))
gs = fig.add_gridspec(3, 2, height_ratios=[1.8, 1.2, 1])

# ---- 左上: 轨迹全景 ----
ax1 = fig.add_subplot(gs[0, :])

odom_t = np.array([o['t'] for o in odom_data])
odom_x = np.array([o['x'] for o in odom_data])
odom_y = np.array([o['y'] for o in odom_data])

# 三段分段着色 (仅对有效到达的 goal 着色)
for i, res in enumerate(results):
    g1 = res['goal_t']
    g2 = res['phase2_start']
    g3 = res['phase2_end']
    g4 = min(res['phase3_end'], res['next_goal_t'])

    mask_fast = (odom_t >= g2) & (odom_t <= g3)
    mask_fine = (odom_t > g3) & (odom_t <= g4)
    mask_prep = (odom_t >= g1) & (odom_t < g2)

    if np.any(mask_fast):
        ax1.plot(odom_x[mask_fast], odom_y[mask_fast], color=C_FAST, linewidth=2.5, alpha=0.9)
    if np.any(mask_fine):
        ax1.plot(odom_x[mask_fine], odom_y[mask_fine], color=C_FINE, linewidth=2.5, alpha=0.9)
    if np.any(mask_prep):
        ax1.plot(odom_x[mask_prep], odom_y[mask_prep], color=C_PREPARE, linewidth=1.5, alpha=0.5, linestyle='--')

# 全局规划路径
for i, res in enumerate(results):
    best_plan = None
    for gp in global_paths:
        if gp['t'] is None: continue
        if abs(gp['t'] - res['goal_t']) < 2.0:
            if best_plan is None or abs(gp['t'] - res['goal_t']) < abs(best_plan['t'] - res['goal_t']):
                best_plan = gp
    if best_plan and best_plan['poses']:
        px = [p[0] for p in best_plan['poses']]
        py = [p[1] for p in best_plan['poses']]
        lbl = 'Global plan' if i == 0 else ''
        ax1.plot(px, py, color=C_GLOBAL, linewidth=1.2, alpha=0.5, linestyle=':', label=lbl)

# TEB 局部路径
for i, res in enumerate(results):
    teb_samples = [tp for tp in teb_locals if tp['t'] is not None
                   and res['phase2_start'] <= tp['t'] <= min(res['phase3_end'], res['next_goal_t'])]
    step = max(1, len(teb_samples) // 6)
    for j, tp in enumerate(teb_samples[::step]):
        if tp['poses'] and len(tp['poses']) >= 2:
            px = [p[0] for p in tp['poses']]
            py = [p[1] for p in tp['poses']]
            lbl = 'TEB local' if (i == 0 and j == 0) else ''
            ax1.plot(px, py, color=C_TEB, linewidth=0.8, alpha=0.35, label=lbl)

# 起点/目标标注
for i, res in enumerate(results):
    ax1.scatter(*res['start'], color=C_START, s=120, marker='o',
                edgecolors='white', linewidths=1.5, zorder=5)
    ax1.scatter(*res['goal'], color=C_GOAL, s=120, marker='X',
                edgecolors='white', linewidths=1.5, zorder=5)
    ax1.annotate(f'G{i+1}', res['goal'], textcoords="offset points",
                 xytext=(10, 10), fontsize=10, color=C_GOAL, fontweight='bold')

ax1.scatter([odom_x[0]], [odom_y[0]], color='white', s=60, marker='s', zorder=5)
ax1.annotate('Origin', (odom_x[0], odom_y[0]), textcoords="offset points",
             xytext=(5, -15), fontsize=9, color='white')

ax1.set_xlabel('X (m)', fontsize=12)
ax1.set_ylabel('Y (m)', fontsize=12)
ax1.set_title('[MAP]  ABOT 导航轨迹全景 — 实际轨迹(分段着色) + 全局规划 + TEB 局部路径', fontsize=14, fontweight='bold')

# 自定义图例
legend_elements = [
    plt.Line2D([0],[0], color=C_FAST, linewidth=2.5, label=f'快速路 (>=0.16m/s)'),
    plt.Line2D([0],[0], color=C_FINE, linewidth=2.5, label=f'微调 (<0.16m/s)'),
    plt.Line2D([0],[0], color=C_PREPARE, linewidth=1.5, linestyle='--', label='准备'),
    plt.Line2D([0],[0], color=C_GLOBAL, linewidth=1.2, linestyle=':', label='全局规划'),
    plt.Line2D([0],[0], color=C_TEB, linewidth=0.8, alpha=0.35, label='TEB 局部路径'),
]
ax1.legend(handles=legend_elements, loc='upper left', fontsize=8, framealpha=0.8)
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.2, linestyle='--')

# ---- 中左: Goal 累计里程 vs 直线距离 ----
ax2 = fig.add_subplot(gs[1, 0])

odom_dist_cum = np.zeros(len(odom_data))
for j in range(1, len(odom_data)):
    odom_dist_cum[j] = odom_dist_cum[j-1] + np.sqrt(
        (odom_data[j]['x'] - odom_data[j-1]['x'])**2 +
        (odom_data[j]['y'] - odom_data[j-1]['y'])**2)

# 取最后一个完整 goal 的详细视图 (Goal 3 实际在动)
res_detail = results[-1]  # Goal 3
t0 = res_detail['goal_t']
mask_g = (odom_t >= t0 - 0.5) & (odom_t <= res_detail['phase3_end'] + 0.5)
t_rel = odom_t[mask_g] - t0
dist_from_goal_start = odom_dist_cum[mask_g] - (odom_dist_cum[mask_g][0] if any(mask_g) else 0)

if len(t_rel) > 0:
    ax2.plot(t_rel, dist_from_goal_start, color=C_ACTUAL, linewidth=2, label='实际累计里程')

ax2.axhline(y=res_detail['straight_dist'], color=C_GLOBAL, linestyle=':', linewidth=1.5,
            label=f'直线距离 {res_detail["straight_dist"]:.2f}m')

if res_detail['t_prepare'] > 0.01:
    ax2.axvspan(0, res_detail['t_prepare'], alpha=0.12, color=C_PREPARE, label='准备')
ax2.axvspan(res_detail['t_prepare'], res_detail['t_prepare'] + res_detail['t_fast'],
            alpha=0.12, color=C_FAST, label='快速路')
ax2.axvspan(res_detail['t_prepare'] + res_detail['t_fast'],
            res_detail['t_total'], alpha=0.12, color=C_FINE, label='微调')

ax2.set_xlabel('时间 (s)', fontsize=11)
ax2.set_ylabel('累计里程 (m)', fontsize=11)
ax2.set_title(f'[METER] Goal {res_detail["goal_idx"]} 实际行驶里程 vs 直线距离', fontsize=12, fontweight='bold')
ax2.legend(loc='upper left', fontsize=8)
ax2.grid(True, alpha=0.2, linestyle='--')

# ---- 中右: 各 Goal 三段时长堆叠柱状图 ----
ax3 = fig.add_subplot(gs[1, 1])

goal_labels = []
for r in results:
    flag = ' [!!]' if r['preempted'] else ''
    goal_labels.append(f'Goal {r["goal_idx"]}\n{r["straight_dist"]:.2f}m{flag}')

x_pos = np.arange(len(results))
width = 0.55

b1 = ax3.bar(x_pos, [r['t_prepare'] for r in results], width, color=C_PREPARE, label='准备')
b2 = ax3.bar(x_pos, [r['t_fast'] for r in results], width,
             bottom=[r['t_prepare'] for r in results], color=C_FAST, label='快速路 (>=0.16m/s)')
b3 = ax3.bar(x_pos, [r['t_fine'] for r in results], width,
             bottom=[r['t_prepare'] + r['t_fast'] for r in results], color=C_FINE, label='微调 (<0.16m/s)')

for i, r in enumerate(results):
    y = 0
    if r['t_prepare'] > 0.1:
        ax3.text(i, r['t_prepare']/2, f"{r['t_prepare']:.1f}s", ha='center', va='center', fontsize=8, color='white')
    if r['t_fast'] > 0.1:
        ax3.text(i, r['t_prepare'] + r['t_fast']/2, f"{r['t_fast']:.1f}s", ha='center', va='center', fontsize=8, color='white')
    if r['t_fine'] > 0.1:
        ax3.text(i, r['t_prepare'] + r['t_fast'] + r['t_fine']/2, f"{r['t_fine']:.1f}s", ha='center', va='center', fontsize=8, color='white')
    if r['t_total'] > 0:
        ax3.text(i, r['t_total'] + 0.5, f'Σ {r["t_total"]:.1f}s', ha='center', fontsize=10, fontweight='bold', color='white')

ax3.set_xticks(x_pos)
ax3.set_xticklabels(goal_labels)
ax3.set_ylabel('耗时 (s)', fontsize=11)
ax3.set_title('[TIME]  三段计时分布', fontsize=12, fontweight='bold')
ax3.legend(loc='upper right', fontsize=8)
ax3.grid(True, alpha=0.2, axis='y', linestyle='--')

# ---- 左下: 速度曲线 ----
ax4 = fig.add_subplot(gs[2, 0])

cmd_t = np.array([c['t'] for c in cmd_data])
cmd_v = np.array([np.sqrt(c['lx']**2 + c['ly']**2) for c in cmd_data])
cmd_w = np.array([abs(c['az']) for c in cmd_data])

t0_all = results[0]['goal_t']
t_end_all = results[-1]['phase3_end']
mask_v = (cmd_t >= t0_all - 0.5) & (cmd_t <= t_end_all + 1.0)
t_v_rel = cmd_t[mask_v] - t0_all

ax4.plot(t_v_rel, cmd_v[mask_v], color=C_ACTUAL, linewidth=1.8, label='合速度 (m/s)')
ax4.plot(t_v_rel, cmd_w[mask_v], color=C_TEB, linewidth=1.5, alpha=0.7, label='|ω| (rad/s)')
ax4.axhline(y=V_THRESHOLD, color=C_FAST, linestyle='--', linewidth=1.2, alpha=0.8,
            label=f'快速路阈值 {V_THRESHOLD} m/s')

# goal 到达时刻竖线
for res in results:
    ax4.axvline(x=res['goal_t'] - t0_all, color=C_GOAL, linestyle='-', linewidth=1.2, alpha=0.6)
    ax4.text(res['goal_t'] - t0_all, ax4.get_ylim()[1] * 0.95, f'G{res["goal_idx"]}',
             ha='center', fontsize=8, color=C_GOAL)

ax4.set_xlabel('时间 (s)', fontsize=11)
ax4.set_ylabel('速度', fontsize=11)
ax4.set_title('[SPEED] 全程速度曲线 + Goal 发送时刻', fontsize=12, fontweight='bold')
ax4.legend(loc='upper right', fontsize=8)
ax4.grid(True, alpha=0.2, linestyle='--')

# ---- 右下: TEB 曲率 ----
ax5 = fig.add_subplot(gs[2, 1])

for i, res in enumerate(results):
    colors = [C_FAST, C_TEB, C_PREPARE]
    best_local = None
    for tp in teb_locals:
        if tp['t'] is None: continue
        mid_t = res['phase2_start'] + (min(res['phase3_end'], res['next_goal_t']) - res['phase2_start']) / 2
        window = max(1.0, (min(res['phase3_end'], res['next_goal_t']) - res['phase2_start']) / 3)
        if abs(tp['t'] - mid_t) < window and len(tp['poses']) > 3:
            best_local = tp
            break
    if best_local is None:
        continue

    px = np.array([p[0] for p in best_local['poses']])
    py = np.array([p[1] for p in best_local['poses']])
    if len(px) < 3:
        continue

    curvatures = []
    arc_positions = [0]
    for j in range(1, len(px)):
        arc_positions.append(arc_positions[-1] + np.sqrt((px[j]-px[j-1])**2 + (py[j]-py[j-1])**2))

    for j in range(1, len(px) - 1):
        x1, y1 = px[j] - px[j-1], py[j] - py[j-1]
        x2, y2 = px[j+1] - px[j], py[j+1] - py[j]
        angle1 = np.arctan2(y1, x1)
        angle2 = np.arctan2(y2, x2)
        d_angle = angle2 - angle1
        while d_angle > np.pi: d_angle -= 2 * np.pi
        while d_angle < -np.pi: d_angle += 2 * np.pi
        ds = max(np.sqrt((px[j]-px[j-1])**2 + (py[j]-py[j-1])**2), 0.001)
        curvatures.append(abs(d_angle / ds))

    if curvatures:
        ax5.plot(arc_positions[1:-1], curvatures, alpha=0.7, linewidth=1.5,
                 label=f'Goal {i+1} TEB 曲率', color=colors[i % len(colors)])

ax5.set_xlabel('弧长 (m)', fontsize=11)
ax5.set_ylabel('曲率 |k| (rad/m)', fontsize=11)
ax5.set_title('[CURVE] TEB 局部路径曲率分布 (平滑度指标)', fontsize=12, fontweight='bold')
ax5.legend(loc='upper right', fontsize=8)
ax5.grid(True, alpha=0.2, linestyle='--')

plt.tight_layout(pad=2)
out_path = os.path.join(OUT_DIR, 'nav_analysis.png')
fig.savefig(out_path, dpi=150, facecolor='#0d1117', edgecolor='none', bbox_inches='tight')
print(f"\n✅ 图表已保存: {out_path}")

# ============== JSON 输出 ==============
stats = {
    'bag_duration_s': round(odom_data[-1]['t'] - odom_data[0]['t'], 1),
    'goals': [],
}
for r in results:
    avg_spd = round(r['fast_dist'] / r['t_fast'], 3) if r['t_fast'] > 0 else 0
    eff = round(r['straight_dist'] / r['fast_dist'] * 100, 1) if r['fast_dist'] > 0 else 0
    stats['goals'].append({
        'idx': r['goal_idx'],
        'preempted': r['preempted'],
        'start_xy': [round(x, 3) for x in r['start']],
        'goal_xy': [round(x, 3) for x in r['goal']],
        'straight_dist_m': round(r['straight_dist'], 3),
        't_prepare_s': round(r['t_prepare'], 2),
        't_fast_s': round(r['t_fast'], 2),
        't_fine_s': round(r['t_fine'], 2),
        't_total_s': round(r['t_total'], 2),
        'fast_dist_m': round(r['fast_dist'], 3),
        'avg_speed_fast_mps': avg_spd,
        'path_efficiency_pct': eff,
    })

json_path = os.path.join(OUT_DIR, 'nav_stats.json')
with open(json_path, 'w', encoding='utf-8') as f:
    json.dump(stats, f, ensure_ascii=False, indent=2)
print(f"✅ 统计 JSON: {json_path}")

# ============== 终报 ==============
print("\n" + "=" * 60)
print("[ SUM] 终报摘要")
print("-" * 60)
for r in results:
    avg_spd = r['fast_dist'] / r['t_fast'] if r['t_fast'] > 0 else 0
    eff = r['straight_dist'] / r['fast_dist'] * 100 if r['fast_dist'] > 0 else 0
    flag = ' [抢占]' if r['preempted'] else ''
    print(f"Goal {r['goal_idx']}{flag}: 总{r['t_total']:.1f}s "
          f"准备{r['t_prepare']:.1f}s | 快速{r['t_fast']:.1f}s({r['fast_dist']:.2f}m,{avg_spd:.2f}m/s) | 微调{r['t_fine']:.1f}s")
    print(f"         直线={r['straight_dist']:.2f}m  实际={r['fast_dist']:.2f}m  效率={eff:.0f}%")
print("=" * 60)
