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
  [auto_task_runner / goal_nav / ten_point_*]  ──► /cmd_vel ──► 底盘 MCU
        闭环原语: 只沿 xy 轴平移 + 原地转, 锁航向压漂, 不经 move_base/TEB/DWA
```

**一句话**：EKF 给 `odom→base`，激光定位给 `map→odom`，合成的 `map→base` 喂给运动脚本闭环发 `cmd_vel`。

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
| 定位 | `localize.launch` | rplidar + map_server(comp.yaml) + amcl | `/map`、`map→odom`（**无 move_base**） |
| 抓取 | `vl_locate.launch` + `ZachLab_grasp/grasp.launch` | 相机 + VLM 检测 + 机械臂 | `/vlm_detection`、`/grab` 握手 |

> 历史的 `navigate.launch`（map_server+amcl+**move_base**）已被 `localize.launch` 取代，见下。

---

## 4. 导航方案：为什么绕开 move_base（2026-05-26 定向）

- **问题**：move_base 的局部规划器（TEB/DWA）反应式避障会贴近挡板，而比赛**碰挡板即判负**；且旧编排器 `client.py` 硬依赖 move_base actionlib。
- **方案**：保留**全局规划**（静态地图上确定性算路，无问题），运动改走**轴对齐 cmd_vel 原语**：
  - 读 `map→base_footprint` tf 做 P 控制，L 形分解（先 X 后 Y）+ 全程锁航向 + 原地转。
  - 严格只沿 xy 轴走，不斜穿、不贴挡板，满足硬约束。
- **手动导航**（`goal_nav.py`）：订阅 rviz 2D Nav Goal → 自写 A*（4 连通 + 障碍膨胀）→ 折成 x/y 直线段 → 原语执行。
- ⚠️ **不要再调 TEB/DWA 参数**——属已废弃方向。

---

## 5. 两种运行模式（互斥）

```
建图模式:  bringup + lidar_slam(gmapping)        → 建图 → map_saver 存 comp.pgm/.yaml
定位模式:  bringup + localize(amcl 读 comp.yaml)  → 跑运动脚本
```

建图起点须摆在**比赛原点、车头朝 +x**，使地图原点 = (0,0) = amcl 初值，与运动脚本坐标对齐。

---

## 6. 关键 frame / topic 速查

| 名称 | 含义 |
|------|------|
| `/raw_odom` | 轮速原始里程计（麦轮正运动学） |
| `/odom` | EKF 融合输出，并发 `odom→base_footprint` |
| `/imu/data` | Madgwick 滤波后 IMU |
| `/scan` | RPLidar C1（460800） |
| `/map` | map_server(定位) 或 gmapping(建图) |
| `/cmd_vel` | 运动脚本下发，麦轮全向（含 `linear.y` 侧移） |
| `/grab` `/vlm_detection` | 抓放握手（client ↔ gg.py） |

> 命令速查见 [`启动命令速查.md`](启动命令速查.md)；比赛硬约束见 [`比赛规则.md`](比赛规则.md)；脚本索引见 [`../CLAUDE.md`](../CLAUDE.md)。
