# Small Car — ABOT M1 ARM 智能小车

> 基于 ROS 的自主导航 + 视觉抓取 + 机械臂操控实验平台
> 属于 [三仓库生态系统](https://github.com/zhangdashuai968/morning-newspaper/blob/master/WORKFLOW.md) 的工程代码仓库

## 硬件平台

- **底盘**: ABOT M1 ARM 麦克纳姆轮（四轮）
- **计算**: NVIDIA Jetson Nano（Ubuntu 20.04 / ROS Noetic）
- **传感器**: 思岚 RPLidar C1 | Astra RGBD 深度相机 | MPU6050 IMU（9轴）
- **执行器**: 六自由度机械臂（吸盘抓取）
- **连接**: 局域网 SSH（`192.168.36.46` 或 `192.168.43.211`，用户 `abot`）

## 硬件详情

### 网络

| 项目 | 值 |
|------|-----|
| 网段 A IP | `192.168.36.46` |
| 网段 B IP | `192.168.43.211` |
| SSH 用户 | `abot` |
| SSH 密码 | 见环境变量 `CAR_PASSWORD` |
| ROS Master | `http://<小车IP>:11311` |

### 计算平台

| 项目 | 值 |
|------|-----|
| 型号 | NVIDIA Jetson Nano |
| OS | Ubuntu 20.04 |
| ROS 发行版 | Noetic |
| 工作空间 | `~/catkin_ws` |
| 串口设备 | `/dev/abotbase` (底盘), `/dev/abotlidar` (激光) |

### 底盘

| 项目 | 值 |
|------|-----|
| 型号 | ABOT M1 ARM |
| 驱动方式 | 麦克纳姆轮 (四轮) |
| 环境变量 | `ABOTMODEL=x4`, `ABOTBASE=omni`（麦轮全向模式） |
| 底层 MCU | STM32F103RCT6 |
| 通信协议 | rosserial (USB, 115200 bps) |
| 里程计标定 | `linear_scale = 1.014` |

### 传感器

| 传感器 | 型号 | 话题/备注 |
|--------|------|-----------|
| 激光雷达 | 思岚 RPLidar C1 | `/scan`, 波特率 460800 |
| 深度相机 | Astra RGBD | `/camera/rgb/image_raw`, `/camera/depth/points` |
| IMU | MPU6050 (板载) | `/imu/data`, Madgwick 滤波后 |
| 里程计 | 轮式编码器 → EKF 融合 | `/odom` (EKF 融合后) |

## 已知 Bug

| # | 描述 | 状态 |
|----|------|------|
| 1 | **轨迹漂移** — 麦轮可能存在滚子打滑或里程计标定不准，详见 `TROUBLESHOOTING.md` | 排查中 |
| 2 | **双 WiFi 网段 IP 不固定** — 每次开机可能分配到 36.x 或 43.x 网段 | 需手动 ping 确认 |
| 3 | **编码文件中文乱码** — 车上部分文件为 GBK 编码，拉到 Windows 后需转换 | 逐步修复 |
| 4 | **AMCL 初始位姿不匹配** — 启动时如果车不在 (0.6, -0.4) 需要重新设 initialpose | 已配置默认值 |

> 详见 `TROUBLESHOOTING.md` 和 `reports/调参极限分析报告.md`

## 启动流程

```bash
# 1. 基础驱动 + 里程计 + EKF
roslaunch abot bringup.launch

# 2. 地图 + 定位 + move_base
roslaunch abot navigate.launch

# 3. [可选] 视觉定位 + 抓取
roslaunch vl_locate vl_locate.launch
roslaunch ZachLab_grasp grasp.launch

# 4. 执行任务
rosrun abot auto_task_runner.py waypoints.yaml
```

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
│   ├── waypoints_test.yaml # 测试航线
│   ├── patrol_run.sh       # 启动巡逻任务
│   ├── auto_task_runner.py # 自动任务调度
│   ├── nav_test.py         # 前进 1m 测试
│   ├── rotate_test.py      # 旋转 90° 测试
│   ├── ssh-car.py          # SSH 远程控制工具
│   ├── auto-sync.sh        # GitHub 自动同步
│   ├── sync-once.sh/.bat   # 手动同步脚本
│   └── start_map.sh        # 一键建图启动
├── maps/                   # 场地地图
│   ├── house.pgm
│   └── house.png
├── logs/                   # 真机调试日志
│   ├── 模板_调试日志.md
│   └── 小车调节日志.txt
├── reports/                # 会话报告
│   ├── README.md
│   ├── 模板_人工报告.md
│   ├── 模板_AI报告.md
│   └── 调参极限分析报告.md
├── CLAUDE.md               # AI 助手规则
├── TROUBLESHOOTING.md      # 踩坑手册
└── README.md
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
python scripts/ssh-car.py          # 交互式 shell
python scripts/ssh-car.py "ls"     # 执行单条命令
```

### 2. 启动巡逻

```bash
# SSH 到车上后
cd ~/catkin_ws
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

`maps/house.pgm` / `maps/house.png` — 场地图，可直接用 ROS `map_server` 加载。

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

Public repository — ABOT M1 ARM 智能小车 ROS 工作区。
