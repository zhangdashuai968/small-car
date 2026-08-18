# 系统架构总览

> 一页看懂 small-car 的数据流、tf 树、节点分层与导航方案。新人/Claude Code 上手先读这页。
> 平台：Jetson Nano · Ubuntu 18.04 · ROS Melodic · Python 2.7 · 麦克纳姆轮全向底盘。

---

## 1. 数据流（核心）

```
  轮式编码器 ──/raw_odom──┐
                         ├──► [robot_localization EKF] ──► /odom ──► tf: odom → base_footprint
  MPU6050 ──/imu/data────┘            (融合 raw_odom 的 vx/vy + imu 的 yaw/vyaw)
                                       ★ 航向 100% 来自 IMU, 轮速 yaw 不采

  RPLidar C1 ──/scan──► [gmapping 或 amcl, 二选一] ──► tf: map → odom

  tf: map → base_footprint  (上面两段合成)
        │
        ▼
  [move_base] 全局规划(A*/Dijkstra) + DWA 局部规划 ──► /cmd_vel ──► 底盘 MCU
        航线由 move_base_waypoint_runner / auto_navigation_grasp 按 waypoints.yaml 下发
```

**一句话**：EKF 给 `odom→base`，激光定位给 `map→odom`，合成的 `map→base` 供 move_base 规划，目标点经 DWA 局部规划发 `cmd_vel`。

---

## 2. tf 树

```
map
 └── odom                 ← gmapping(建图) 或 amcl(定位) 发布, 二选一
      └── base_footprint   ← EKF(bringup) 发布
           └── base_link    ← static_transform (bringup)
                ├── laser    ← static (雷达安装位)
                └── imu_link  ← static
```

- `map→odom`：**建图与定位互斥**——gmapping 与 amcl 都发这一段，同一时间只能起一个。
- `odom→base_footprint`：始终由 `bringup.launch` 的 EKF 提供（纯里程计保底脚本只靠这一段）。

---

## 3. 节点 / launch 分层

| 层 | launch | 起的东西 | 提供 |
|----|--------|----------|------|
| 底盘 | `bringup.launch` | abot_base_node + apply_calib + imu_filter_madgwick + ekf | `/odom`、`odom→base`、收 `/cmd_vel` |
| 建图 | `lidar_slam.launch` / `cartographer_slam.launch` | rplidar + gmapping/cartographer | `/map`、`map→odom` |
| 定位+导航 | `navigate.launch` | rplidar + map_server(house.yaml) + amcl + move_base(DWA) | `/map`、`map→odom`、move_base 目标执行 |
| 定位(保底) | `localize.launch` | rplidar + map_server(comp.yaml) + amcl（**无 move_base**） | `/map`、`map→odom` |
| 抓取 | `vl_locate.launch` + `ZachLab_grasp/grasp.launch` | 相机 + VLM 检测 + 机械臂 | `/vlm_detection`、`/grab` 握手 |

> 导航方案沿革见下节：2026-05-26 曾绕开 move_base（`localize.launch` + cmd_vel 原语），06-01 起切回 `navigate.launch`（move_base）。

---

## 4. 导航方案：为什么切回 move_base（2026-06-01 起）

- **沿革**：2026-05-26 曾因 move_base 的局部规划器（TEB/DWA）反应式避障会贴近挡板（比赛**碰挡板即判负**）而绕开它，运动走轴对齐 cmd_vel 原语（`auto_task_runner.py` / `goal_nav.py` / `ten_point_*`，读 `map→base_footprint` tf 做 P 控制，L 形分解 + 锁航向）。
- **2026-06-01 起切回 move_base**：先用 TEB 全向参数调优（`teb_omni_planner_params.yaml`），06-07 起转 DWA（`dwa_omni_planner_params.yaml`：保守中心线参数 + 到点后 yaw 对齐），并配合 AMCL 调优（`amcl.launch`）。
- **主流程**：`navigate.launch`（rplidar + map_server + amcl + move_base），航线 `scripts/waypoints.yaml` 由 `move_base_waypoint_runner.py` / `auto_navigation_grasp.py` 发给 move_base 执行。
- 历史原语脚本保留为**保底**（走 `localize.launch`，无 move_base）。

---

## 5. 两种运行模式（互斥）

```
建图模式:  bringup + lidar_slam(gmapping)        → 建图 → map_saver 存图
导航模式:  bringup + navigate(amcl 读 house.yaml + move_base/DWA)  → 发航线点
保底模式:  bringup + localize(amcl 读 comp.yaml, 无 move_base)     → 原语脚本
```

建图起点须摆在**比赛原点、车头朝 +x**。amcl 初值：`navigate.launch` 用 `amcl.launch` 默认 (0.6, -0.4, 0)；`localize.launch` 覆盖为 (0,0,0)。

---

## 6. 关键 frame / topic 速查

| 名称 | 含义 |
|------|------|
| `/raw_odom` | 轮速原始里程计（麦轮正运动学） |
| `/odom` | EKF 融合输出，并发 `odom→base_footprint` |
| `/imu/data` | Madgwick 滤波后 IMU |
| `/scan` | RPLidar C1（460800） |
| `/map` | map_server(定位) 或 gmapping(建图) |
| `/cmd_vel` | move_base(DWA) 下发，麦轮全向（含 `linear.y` 侧移） |
| `/grab` `/vlm_detection` | 抓放握手（client ↔ gg.py） |

> 命令速查见 [`启动命令速查.md`](启动命令速查.md)；比赛硬约束见 [`比赛规则.md`](比赛规则.md)；脚本索引见 [`../CLAUDE.md`](../CLAUDE.md)。
