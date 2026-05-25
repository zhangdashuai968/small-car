# Small Car — ABOT M1 ARM 智能小车

> 基于 ROS 的自主导航 + 视觉抓取 + 机械臂操控实验平台
> 属于 [三仓库生态系统](https://github.com/zhangdashuai968/morning-newspaper/blob/master/WORKFLOW.md) 的工程代码仓库

## 硬件平台

- **底盘**: ABOT M1 ARM 轮式机器人
- **计算**: NVIDIA Jetson（Ubuntu 20.04 / ROS Noetic）
- **传感器**: LIDAR 激光雷达 | 深度相机 | IMU（9轴）
- **执行器**: 六自由度机械臂（吸盘抓取）
- **连接**: 局域网 SSH（`192.168.36.46`，用户 `abot`）

## 项目结构

```
├── src/                    # ROS 工作区（catkin workspace）
│   ├── lidar/              # 激光雷达驱动与点云处理
│   ├── depth_camera/       # 深度相机驱动
│   ├── jetson_camera/      # Jetson 板载摄像头
│   ├── camera_umd/         # 摄像头 UMD 驱动
│   ├── imu_calib/          # IMU 标定
│   ├── imu_filter_madgwick/ # Madgwick IMU 姿态滤波
│   ├── robot_localization/ # 多传感器融合定位
│   ├── m-explore/          # 自主探索建图
│   ├── opencv_apps/        # OpenCV 视觉应用
│   ├── robot_arm/          # 机械臂控制
│   └── abot_project/       # ABOT M1 项目主逻辑
├── scripts/                # 运行脚本
│   ├── waypoints.yaml      # 巡航线点（抓取/放置动作编排）
│   ├── patrol_run.sh       # 启动巡逻任务
│   ├── start_patrol_clean.py   # 巡逻清理程序
│   ├── test_goal_one.py    # 单目标测试
│   ├── auto_task_runner.py # 自动任务调度
│   ├── auto-sync.sh        # GitHub 自动同步
│   ├── sync-once.sh/.bat   # 手动同步脚本
│   └── waypoints_test.yaml # 测试航线
├── ssh-car.py              # SSH 远程控制工具
├── house.pgm / house.png   # 场地地图
└── 小车调节日志.txt        # 调试记录
```

## 核心功能

| 模块 | 功能 |
|------|------|
| **自主导航** | LIDAR + IMU 融合定位，基于 `waypoints.yaml` 的航线巡逻 |
| **视觉抓取** | 深度相机识别目标物体（葡萄、香蕉、山竹）→ 机械臂抓取 |
| **物体放置** | 导航到投放点，释放物体到指定位置 |
| **探索建图** | `m-explore` 自主探索，构建占据栅格地图 |
| **远程操控** | `ssh-car.py` 远程命令执行 / 交互式 shell |

## 快速开始

### 1. 连接小车

```bash
python ssh-car.py          # 交互式 shell
python ssh-car.py "ls"     # 执行单条命令
```

### 2. 启动巡逻

```bash
# SSH 到车上后
cd ~/robot_ws
./src/patrol_run.sh
```

### 3. 航线配置

编辑 `scripts/waypoints.yaml`：

```yaml
waypoints:
  - name: pickup_grape
    x: -0.137
    y: 2.485
    yaw: 0.0
    action: grab          # grab / place / pass
    object: 葡萄           # 识别目标关键词
```

### 4. 地图查看

`house.pgm` / `house.png` — 场地图，可直接用 ROS `map_server` 加载。

## 环境要求

- ROS Noetic（Ubuntu 20.04）
- Python 3.8+（`paramiko`、`numpy`）
- OpenCV 4.x
- CMake 2.8.3+

## 关联项目

| 项目 | 说明 |
|------|------|
| [abot_arm_learning](https://github.com/zhangdashuai968/abot_arm_learning) | 配套学习路线图：6 阶段 32 实验，从 ROS 基础到 SLAM 到视觉伺服抓取 |
| [morning-newspaper](https://github.com/zhangdashuai968/morning-newspaper) | 每日早报系统，包含学习进度追踪和每日工作日志 |
| [WORKFLOW.md](https://github.com/zhangdashuai968/morning-newspaper/blob/master/WORKFLOW.md) | 三仓库生态系统完整说明 |

> **关系**：`abot_arm_learning` 是"课堂"（学原理、做实验），`small-car` 是"战场"（实际部署、跑航线、真机调试）。`morning-newspaper` 是"闹钟"（每天早上提醒你进度到哪了）。

## 许可证

Private repository — 仅供团队内部使用。
