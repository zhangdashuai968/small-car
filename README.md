# Small Car — ABOT M1 ARM 智能小车

> 基于 ROS 的自主导航 + 视觉抓取 + 机械臂操控竞赛平台。
> 属于[三仓库生态](https://github.com/zhangdashuai968/morning-newspaper/blob/master/WORKFLOW.md)的「战场」仓（真机部署 + 调试）；课堂仓是 [abot_arm_learning](https://github.com/zhangdashuai968/abot_arm_learning)。

`Jetson Nano` · `Ubuntu 18.04` · `ROS Melodic` · `Python 2.7` · `麦克纳姆轮全向底盘`

---

## 📑 文档导航

| 文档 | 用途 |
|------|------|
| [`CLAUDE.md`](CLAUDE.md) | **AI 上手中枢**：目录地图、脚本索引、当前导航方案、约定 |
| [`CONTRIBUTING.md`](CONTRIBUTING.md) | **协作约定**：git 流程、代理、编码/换行、脚本放哪、真机验证 |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | **系统架构**：数据流、tf 树、节点分层、绕开 move_base 的设计 |
| [`docs/比赛规则.md`](docs/比赛规则.md) | **最终目标**：场地、6 点抓放流程、得分、致命终止条件 |
| [`docs/启动命令速查.md`](docs/启动命令速查.md) | 清进程 → 建图 → 存图 → 定位 → 运动执行命令速查 |
| [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) | 踩坑手册（按症状 → 根因 → 解法） |
| [`reports/`](reports) · [`logs/`](logs) | 会话报告 / 调试日志（均有模板） |
| `docs/团队必读文档汇总.html` | 上述核心文档的单文件离线汇总（`python tools/gen_docs.py` 生成） |

---

## 🎯 比赛目标

**室内移动抓取搬运赛**：3.6×3.6 m 场地，起点 → 6 个搬运点（奇数抓 / 偶数放）→ 终点。
⚠️ **碰围栏/挡板 或 30s 不动即立即判负**——这是工程上坚持「只沿 xy 轴走、不漂移、原地转、fail-fast」的硬约束。完整规则见 [`docs/比赛规则.md`](docs/比赛规则.md)。

---

## 🏗️ 架构速览

```
轮速 + IMU ─► [EKF] ─► odom→base_footprint ┐
激光 ─► [gmapping 或 amcl, 二选一] ─► map→odom ┤
                                              ├─► map→base_footprint
                                              ▼
        [运动脚本] 锁航向 cmd_vel 原语(只沿 xy 轴 + 原地转, 绕开 move_base/TEB/DWA)
```

**建图与定位互斥**（都发 `map→odom`，永远二选一）。运动不经局部规划器，避免贴挡板判负。完整 tf 树 / 节点分层见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。

---

## 🚀 快速上手

```bash
# 0. 连接（开发机）
python scripts/ssh-car.py            # 交互式 shell；或 ssh-car.py "命令"

# 1. 比赛主流程（车上，定位 + 闭环运动）
roslaunch abot bringup.launch        # 底盘 + IMU + EKF
roslaunch abot localize.launch       # map_server(comp.yaml) + amcl
rosrun abot ten_point_race.py        # 十点抓放（或 goal_nav.py 手动导航）

# 定位栈不可用时的纯里程计保底（只需 bringup）
rosrun abot ten_point_odom_race.py
```

> 完整建图/存图/定位/运动命令（含清残留进程）见 [`docs/启动命令速查.md`](docs/启动命令速查.md)。脚本索引见 [`CLAUDE.md`](CLAUDE.md)。

---

## 🔧 硬件

| 类别 | 关键信息 |
|------|----------|
| 计算 | NVIDIA Jetson Nano（Tegra X1，L4T R32.6.1，Ubuntu 18.04 / ROS Melodic / Py 2.7） |
| 底盘 | ABOT M1 ARM 麦克纳姆轮（四轮），MCU STM32F103RCT6，rosserial 115200，`linear_scale=1.014` |
| 环境变量 | `ABOTMODEL=x4` `ABOTBASE=omni` `NAV_PATH=teb` `ABOTLIDAR=rplidar` `ABOTIMU=mpu6050` |
| 激光 | 思岚 RPLidar C1 → `/scan`（460800），`/dev/abotlidar` |
| 深度相机 | Astra RGBD → `/camera/rgb/image_raw`、`/camera/depth/points` |
| IMU | MPU6050（板载）→ `/imu/data`（Madgwick 滤波） |
| 里程计 | 轮式编码器 → EKF 融合 → `/odom`（**航向 100% 来自 IMU**） |
| 网络 | SSH `abot@192.168.36.46` 或 `192.168.43.211`（双网段，开机随机；ROS Master `:11311`） |

---

## 📁 仓库结构

```
src/abot_project/   自有 ROS 包(abot 主包 + grasp/vision/speech…)，可改
src/<其余>/         vendored 第三方包(robot_localization/lidar/depth_camera…)，勿改
scripts/            主机侧工具(ssh-car/sync/patrol_run) + 历史测试脚本
src/abot_project/scripts/   新运动脚本(goal_nav/ten_point_*/seven_point_test)
maps/  logs/  reports/  docs/  tools/
```

> 完整目录地图与「脚本放哪、哪份为准」见 [`CLAUDE.md`](CLAUDE.md) 与 [`CONTRIBUTING.md`](CONTRIBUTING.md)。

---

## 🔗 关联项目

| 项目 | 关系 |
|------|------|
| [abot_arm_learning](https://github.com/zhangdashuai968/abot_arm_learning) | 「课堂」：6 阶段 32 实验，ROS 基础 → SLAM → 视觉伺服抓取 |
| [morning-newspaper](https://github.com/zhangdashuai968/morning-newspaper) | 「闹钟」：每日早报 + 学习进度追踪 |

`abot_arm_learning` 学原理，**`small-car` 跑真机**，`morning-newspaper` 提醒进度。

---

## 📄 许可证

Public repository — ABOT M1 ARM 智能小车 ROS 工作区。
