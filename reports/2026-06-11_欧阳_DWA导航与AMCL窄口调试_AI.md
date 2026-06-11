# 2026-06-11 欧阳 DWA 导航与 AMCL 窄口调试 AI 报告

> 此报告由 AI 根据 2026-06-10 夜间到 2026-06-11 凌晨的真车调试过程整理。
> 重点用于下一次会话快速恢复：当前保留哪版参数、哪版已回档、下一步优先查什么。

---

## 元数据（机器解析用）

```yaml
session:
  date: "2026-06-11"
  executor: "欧阳"
  experiment: "NONE"
  duration_minutes: 260
  success: partial

git:
  commit_before: "4944772"
  commit_after: "see pushed commit for this report"
  files_changed:
    - path: "logs/小车调节日志.txt"
      change_type: "modified"
      description: "追加 02:01/02:25/02:37 导航调参、失败版回档和交接结论"
    - path: "src/abot_project/abot/param/navigation/dwa/dwa_omni_planner_params.yaml"
      change_type: "modified"
      description: "将标准 DWA scoring 参数改为中心线保守版，移除无效 weight_* 键"
    - path: "reports/2026-06-11_欧阳_DWA导航与AMCL窄口调试_AI.md"
      change_type: "added"
      description: "本次调试交接报告"

vehicle_state:
  bringup_ok: true
  navigation_ok: true
  slam_ok: false
  arm_ok: "not_tested"
  battery_voltage: 0.0
```

## 执行的操作序列

```bash
# 只读检查
rosnode list
rostopic hz /scan
rosrun tf tf_echo map base_footprint
rosrun dynamic_reconfigure dynparam get /move_base/DWAPlannerROS
rosrun dynamic_reconfigure dynparam get /amcl

# 参数同步/回档
scp src/abot_project/abot/param/navigation/dwa/dwa_omni_planner_params.yaml \
  abot@192.168.43.211:/home/abot/catkin_ws/src/abot_project/abot/param/navigation/dwa/dwa_omni_planner_params.yaml
cp .../dwa_omni_planner_params.yaml.bak-20260611-022257 .../dwa_omni_planner_params.yaml
cp .../amcl.launch.bak-20260611-022257 .../amcl.launch
```

## 最终保留参数

保留的是 02:01 “中心线 DWA”版本，02:25 的“更慢更柔 + beamskip”版本已撤销。

| 文件 | 参数名 | 最终值 | 说明 |
|------|--------|--------|------|
| `dwa_omni_planner_params.yaml` | `max_vel_x/y/trans` | `0.25` | 保留上一版能过 1-9 点的速度 |
| `dwa_omni_planner_params.yaml` | `max_vel_theta` | `0.2` | 不再使用 02:25 的 `0.15` |
| `dwa_omni_planner_params.yaml` | `sim_time` | `1.2` | 缩短预测，减少窄口激进轨迹 |
| `dwa_omni_planner_params.yaml` | `prune_plan` | `true` | 裁掉已走过路径 |
| `dwa_omni_planner_params.yaml` | `path_distance_bias` | `32.0` | 强化贴全局路径中心线 |
| `dwa_omni_planner_params.yaml` | `goal_distance_bias` | `12.0` | 降低切角奔终点倾向 |
| `dwa_omni_planner_params.yaml` | `occdist_scale` | `0.08` | 标准 DWA 支持的障碍代价 |
| `dwa_omni_planner_params.yaml` | `twirling_scale` | `8.0` | 惩罚边走边乱转 |
| `amcl.launch` | `laser_likelihood_max_dist` | `1.5` | 已撤销 02:25 的 `0.7` |
| `amcl.launch` | `do_beamskip` | 未启用 | 02:25 启用后表现更差，已回档 |

## 实跑结果摘要

- DWA 标准 scoring 参数修正后，真车 nav-only 有明显进步。
- 12 点路线中，`1-9/12` 点通过，两个新增 pass 点都实际发挥作用。
- 到点后 yaw 对齐正常出现，多处日志有 `yaw aligned`。
- 最终仍在 `[10/12] point_08_grab (-2.298, -3.067, yaw=0.851)` 失败。
- move_base 状态为 `ABORTED`，日志文本为 `Failed to find a valid plan. Even after executing recovery behaviors.`
- 现场观察：出窄处 AMCL 会发散性歪掉；02:25 尝试更慢更柔后，车甚至主动贴向边缘，表现明显倒退。

## 已回档内容

02:25 曾尝试：

- DWA 降到 `max_vel_x/y/trans=0.18`、`max_vel_theta=0.15`；
- 显式设置 `acc_lim_x/y=0.35`、`acc_lim_trans=0.12`、`acc_lim_theta=0.6`；
- AMCL 设置 `laser_likelihood_max_dist=0.7`；
- AMCL 开启 `do_beamskip=true`。

该方案已因现场表现差回档。失败版备份在车上：

```text
dwa_omni_planner_params.yaml.bak-before-revert-20260611-023350
amcl.launch.bak-before-revert-20260611-023350
```

02:01 版本备份：

```text
dwa_omni_planner_params.yaml.bak-20260611-015916
```

## 异常记录

### 1. point_08_grab 仍不可规划

- **症状**：1-9 点通过后，第 10 点 `point_08_grab` ABORT。
- **排查路径**：读 `/move_base/status`、`rosout.log`、当前 TF、AMCL pose、`/scan` 频率。
- **根因判断**：目标点或其周围仍处于障碍/膨胀/局部代价不友好区域；AMCL 出窄后偏移会进一步放大这个问题。
- **下一步**：优先检查 `point_08_grab` 在 global/local costmap 中的落点，不再盲目调 DWA 速度。

### 2. AMCL 出窄后偏移

- **症状**：RViz 中红色 `/scan` 在出窄处与地图边界产生角度/平移错位。
- **已试方案**：增大 AMCL 粒子和 odom 噪声、标准化 DWA 中心线参数、短暂尝试 beamskip。
- **结论**：02:25 进一步收紧 AMCL + 降速方案会导致贴边，已撤销。
- **下一步**：从路径几何和目标点可达性查起，必要时在 `point_07_pass -> point_08_grab` 之间加一个更安全的过渡点。

### 3. ROS/dynamic_reconfigure 状态不稳定

- **症状**：多次 `dynparam set/get` 超时，期间 ROS master 一度不可通信。
- **处理**：清理残留 `dynamic_reconfigure/dynparam` helper，文件层面已回档。
- **下一步**：下一轮测试前重启 `bringup.launch` 与 `navigate.launch`，不要依赖上轮 live 参数状态。

## 传感器/硬件状态

| 设备 | 状态 | 备注 |
|------|------|------|
| RPLIDAR 激光雷达 | OK | `/scan_raw` 与 `/scan` 均曾恢复到约 10Hz |
| TF / EKF | OK | 底盘 bringup 后 `map -> base_footprint`、`base_footprint -> laser` 正常 |
| move_base / DWA | DEGRADED | 能跑 1-9 点，point_08 仍 ABORT |
| AMCL | DEGRADED | 可收敛，但出窄处仍会偏移 |
| 机械臂 / VLM | not_tested | 本轮主要跑 nav-only，不验证抓放 |

## Review 待修问题

- [ ] `move_base_waypoint_runner.py` 中断/异常时需要 `cancel_goal()`、发零速、`/grab=False`、舵机 home。
- [ ] runner 默认 timeout 仍高于 30s 规则风险，需要降到 25s 左右或实现进度 watchdog。
- [ ] `costmap_x4_params.yaml` 中 footprint 放大后 inflation 半径偏小，需重新核算，不能只为通过窄口继续硬砍。
- [ ] `point_08_grab` 落点与其 7x7 邻域曾显示靠近障碍/未知区，需用 costmap/RViz 复核。

## 下一次会话建议

1. 重启 `bringup.launch` 与 `navigate.launch`，确认 `/scan` 10Hz、TF 正常、DWA 参数为 02:01 中心线版本。
2. 重新 RViz `2D Pose Estimate`，只测试 `point_07_pass -> point_08_grab` 或用 `make_plan` 检查该段。
3. 优先看 global/local costmap 中 `point_08_grab` 是否落在膨胀或障碍边界，而不是继续调 AMCL。
4. 如 `point_08_grab` 落点不可用，新增一个 `point_07_08_pass` 或微调 `point_08_grab` 到自由区。
5. 修 review 中 runner 安全退出问题后，再跑完整抓放流程。
