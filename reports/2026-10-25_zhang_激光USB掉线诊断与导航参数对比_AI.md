# 2026-10-25 会话报告：后台清理 + 激光 USB 掉线诊断 + 车仓导航参数对比（AI）

## 概述
远程接入小车（10.66.73.46，原厂 2024-11 TF 卡）完成三件事：两轮后台进程清理、激光无数据的根因定位（物理掉线而非软件）、车上实况参数与本仓库/GitHub 的全量差异对比。

## Git 状态
- 会话起点 HEAD：`b9cda74`（本地领先 origin/main 4 commit：defc712 / a719eb7 / 0b1fb1c / b9cda74，此前因 gh token 失效未推）
- 本机 vs GitHub：param/launch 目录 `git diff origin/main` 零差异
- 本次提交：restore_car.sh tar 快照模式（chore）+ 本报告与日志补记（docs），随后 push origin main

## 硬件状态
| 项 | 状态 |
|---|---|
| 系统 | 中途重启过一次（15:33 开机），内核 4.9.253-tegra，磁盘 91%（剩 2.7G，偏紧） |
| 底盘串口 | 正常。CH340 → ttyUSB1 → `/dev/abotbase`，arduino_serial_node 活，/odom 稳定 20Hz |
| **激光雷达** | **物理掉线**。lsusb 无 CP210x(10c4:ea60)；dmesg 17:28 usb 1-2.1+1-2.3 两设备掉线，底盘回来、雷达未回；udev 规则正常（10c4:ea60→/dev/abotlidar）。需现场插回 USB |
| 手臂板 | STM32F407 已枚举（ttyACM0 → /dev/abotarm_left） |
| ROS 栈 | 队友经 NoMachine 起 bringup.launch + navigate.launch（13 节点）；rplidarNode 随 launch 拉起后因设备不存在死亡，其余正常 |

## 验证结果（证据链）
1. `/scan`、`/scan_raw` 均 no new messages → 排除滤波链问题
2. rplidarNode 参数（serial_port=/dev/abotlidar, 460800）在 param server 注册但节点不在 rosnode list → 启动时设备缺失导致死亡
3. lsusb 全表仅 STM32F407 / CH340 / 蓝牙 / Realtek hub，无任何雷达芯片 → 物理层缺席
4. dmesg 两设备掉线时间（17:28）与 navigate 启动（17:27）吻合 → 掉线即失效起点

**雷达插回后的补启命令**（无需重启 navigate）：
```bash
rosrun rplidar_ros rplidarNode __name:=rplidarNode /scan:=/scan_raw
```

## 参数对比结论（车 = 原厂卡实况 vs 仓库 = 本机/GitHub）
- 车上确认：`base_local_planner = dwa_local_planner/DWAPlannerROS`（NAV_PATH=dwa），ABOTMODEL=x4
- 全仓 diff 排除 CRLF 换行符假差异后，真实内容差异 **12 个文件**：
  - `costmap_x4_params.yaml`：footprint 0.22×0.26→0.36×0.30、inflation_radius 0.2→0.0875、cost_scaling_factor 3.0→3.5、obstacle_range 3.0→2.5
  - `dwa_diff/ackm_planner_params.yaml`：occdist_scale 0.02→0.05
  - `dwa_omni/teb_omni_planner_params.yaml`：整文件重写（限速 0.25m/s 全向版）
  - `common/move_base_params.yaml`：controller_patience 15→5（fail-fast）
  - `slam_gmapping.xml`：高频建图（map_update_interval 15→2s，地图 ±1→±15m）
  - `cartographer/rplidar_lds.lua`：修 TF 双发冲突（provide_odom_frame true→false）
  - `imu_calib.yaml`：标定数据不同（**机器相关，勿盲覆盖**）
  - `bringup.launch`：madgwick 加 gain 0.3
  - 仓库独有：auto_map_save.launch、navigate2.launch

## 遗留
1. 雷达 USB 现场插回（人在车上操作）→ 插回后按上面命令补启 rplidarNode
2. IMU 标定哪份对当前 IMU 准：建议车上重标定后定夺，而非直接覆盖
3. 12 文件是否整体同步上车：待队内决定（比赛方案本身绕开 move_base/DWA，见 AGENTS.md）
4. 磁盘 91%：下次上车清理 ~/.ros/log 与旧 bag

## 本会话无代码/参数文件改动
仅调查与记录；仓库侧改动为本报告、日志补记、restore_car.sh 的 tar 快照支持。
