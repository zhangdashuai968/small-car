#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
从 rosbag CSV 提取导航语义阶段耗时: 准备/快速路/微调
用法: python analyze_nav_stages.py [bags_dir]
"""
import csv, math, sys, os
from collections import defaultdict

if __name__ == '__main__':
    if len(sys.argv) > 1:
        BAGS = sys.argv[1]
    else:
        BAGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'bags')

def load_csv(path):
    with open(path, 'r') as f:
        return list(csv.DictReader(f))

# --- 1. Parse Goals ---
goals = load_csv(os.path.join(BAGS, 'goal.csv'))
print('=' * 72)
print('  >> Target Goals')
print('=' * 72)
goal_list = []
t0 = None  # first timestamp reference
for g in goals:
    ts = float(g['%time'])
    if t0 is None:
        t0 = ts
    x = float(g['field.goal.target_pose.pose.position.x'])
    y = float(g['field.goal.target_pose.pose.position.y'])
    oz = float(g['field.goal.target_pose.pose.orientation.z'])
    ow = float(g['field.goal.target_pose.pose.orientation.w'])
    yaw = 2 * math.atan2(oz, ow)
    gid = g['field.goal_id.id'].split('-')[-1][:8]
    goal_list.append({'ts': ts, 'x': x, 'y': y, 'yaw': yaw, 'id': gid})
    print("  G-{}  t={:.1f}s  pos=({:.2f}, {:.2f})  yaw={:.1f} deg".format(
          gid, ts-t0, x, y, math.degrees(yaw)))

# --- 2. Parse Status transitions ---
status = load_csv(os.path.join(BAGS, 'status.csv'))
STATUS_NAMES = {0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED', 4: 'ABORTED', 5: 'REJECTED'}

print()
print('=' * 72)
print('  >> Status Transitions')
print('=' * 72)
last_gid = None
last_st = None
transitions = []
for s_row in status:
    ts = float(s_row['%time'])
    gid = s_row['field.status_list0.goal_id.id']
    st = int(s_row['field.status_list0.status'])
    text = s_row['field.status_list0.text']
    if gid != last_gid or st != last_st:
        sn = STATUS_NAMES.get(st, '?{}'.format(st))
        short_id = gid.split('-')[-1][:8] if '-' in gid else gid[:8]
        transitions.append({'ts': ts, 'gid': gid, 'status': st, 'sn': sn, 'text': text})
        print("  t={:.1f}s  goal={}  {:>10s}  \"{}\"".format(ts-t0, short_id, sn, text))
        last_gid = gid
        last_st = st

# --- 3. Match Goals to Status ---
goal_ids = {}
for g in goal_list:
    goal_ids[g['id']] = g

goal_phases = []
phase = None
for t in transitions:
    short_id = t['gid'].split('-')[-1][:8] if '-' in t['gid'] else t['gid'][:8]
    if t['status'] == 0:  # PENDING
        if phase:
            goal_phases.append(phase)
        phase = {'gid': short_id, 't_pending': t['ts'], 't_active': None, 't_success': None, 't_abort': None}
    elif phase and t['status'] == 1:
        phase['t_active'] = t['ts']
    elif phase and t['status'] == 3:
        phase['t_success'] = t['ts']
        goal_phases.append(phase)
        phase = None
    elif phase and t['status'] == 4:
        phase['t_abort'] = t['ts']
        goal_phases.append(phase)
        phase = None
if phase:
    goal_phases.append(phase)

# --- 4. Read cmd_vel ---
cmd_vel = load_csv(os.path.join(BAGS, 'cmd_vel.csv'))
v_data = []
for c in cmd_vel:
    ts = float(c['%time'])
    vx = float(c['field.linear.x'])
    vy = float(c['field.linear.y'])
    vz = float(c['field.angular.z'])
    body_sp = math.hypot(vx, vy)
    v_data.append({'ts': ts, 'vx': vx, 'vy': vy, 'vz': vz, 'body_sp': body_sp})

V_LIN = 0.80
FAST_THRESH = 0.8 * V_LIN

# --- 5. Per-goal stage analysis ---
print()
print('=' * 72)
print('  >> Per-Goal Stage Timing')
print('=' * 72)

for gp in goal_phases:
    if not gp['t_success'] and not gp['t_abort']:
        continue
    t_start = gp['t_pending']
    t_end = gp['t_success'] or gp['t_abort']

    seg_vel = [v for v in v_data if t_start <= v['ts'] <= t_end]
    if not seg_vel:
        continue

    # Preparation: PENDING -> ACTIVE
    t_prep = (gp['t_active'] - t_start) if gp['t_active'] else 0

    # Fast vs Fine: based on body-frame speed
    t_fast = 0.0
    t_fine = 0.0
    last_ts = seg_vel[0]['ts']
    for v in seg_vel:
        dt = v['ts'] - last_ts
        if dt <= 0:
            dt = 0.05
        elif dt > 0.5:
            last_ts = v['ts']
            continue
        if v['body_sp'] < 0.01:
            t_fine += dt  # stationary -> rotation, belongs to fine stage
        elif v['body_sp'] >= FAST_THRESH:
            t_fast += dt
        else:
            t_fine += dt
        last_ts = v['ts']

    t_total = t_end - t_start

    g_info = goal_ids.get(gp['gid'], None)
    if g_info:
        print("\n  [Goal-{}]".format(gp['gid']))
        print("     target: ({:.2f}, {:.2f})  yaw={:.0f} deg".format(g_info['x'], g_info['y'], math.degrees(g_info['yaw'])))
    else:
        print("\n  [Goal-{}]".format(gp['gid']))

    print("     Prepare: {:6.2f}s  ({:5.1f}%)  -- global plan + costmap update".format(t_prep, t_prep/t_total*100))
    print("     Fast:    {:6.2f}s  ({:5.1f}%)  -- body_sp >= {:.1f} m/s".format(t_fast, t_fast/t_total*100, FAST_THRESH))
    print("     Fine:    {:6.2f}s  ({:5.1f}%)  -- slowdown + rotation".format(t_fine, t_fine/t_total*100))
    print("     Total:   {:6.2f}s  (100.0%)".format(t_total))

# --- 6. Summary ---
print()
print('=' * 72)
print('  >> Summary')
print('=' * 72)

total_prep = 0; total_fast = 0; total_fine = 0; total_all = 0
for gp in goal_phases:
    if not gp['t_success'] and not gp['t_abort']:
        continue
    t_start = gp['t_pending']
    t_end = gp['t_success'] or gp['t_abort']
    seg_vel = [v for v in v_data if t_start <= v['ts'] <= t_end]
    if not seg_vel:
        continue

    t_prep = (gp['t_active'] - t_start) if gp['t_active'] else 0
    t_fast = 0.0
    t_fine = 0.0
    last_ts = seg_vel[0]['ts']
    for v in seg_vel:
        dt = v['ts'] - last_ts
        if dt <= 0: dt = 0.05
        elif dt > 0.5: last_ts = v['ts']; continue
        if v['body_sp'] < 0.01:
            t_fine += dt
        elif v['body_sp'] >= FAST_THRESH:
            t_fast += dt
        else:
            t_fine += dt
        last_ts = v['ts']

    total_prep += t_prep
    total_fast += t_fast
    total_fine += t_fine
    total_all += (t_end - t_start)

if total_all > 0:
    print("  Prepare total: {:6.2f}s  ({:5.1f}%)".format(total_prep, total_prep/total_all*100))
    print("  Fast total:    {:6.2f}s  ({:5.1f}%)".format(total_fast, total_fast/total_all*100))
    print("  Fine total:    {:6.2f}s  ({:5.1f}%)".format(total_fine, total_fine/total_all*100))
    print("  Nav total:     {:6.2f}s".format(total_all))

    print()
    print('  >> Suggestions:')
    prep_pct = total_prep / total_all
    fine_pct = total_fine / total_all
    fast_pct = total_fast / total_all
    if prep_pct > 0.15:
        print("    [!] Prepare {:2.0f}% > 15%: planner too slow, consider smaller costmap".format(prep_pct*100))
    else:
        print("    [OK] Prepare {:2.0f}% OK".format(prep_pct*100))
    if fine_pct > 0.30:
        print("    [!] Fine {:2.0f}% > 30%: TEB too conservative, increase xy_goal_tolerance".format(fine_pct*100))
    else:
        print("    [OK] Fine {:2.0f}% OK".format(fine_pct*100))
    if fast_pct < 0.40:
        print("    [!] Fast {:2.0f}% < 40%: too much slowdown, check obstacle inflation".format(fast_pct*100))
    elif 0.5 < fast_pct < 0.7:
        print("    [OK] Fast {:2.0f}% in good range".format(fast_pct*100))
    else:
        print("    Fast {:2.0f}%".format(fast_pct*100))
else:
    print("  No valid navigation phases found!")
