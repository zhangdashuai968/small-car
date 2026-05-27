# 小车调试踩坑手册

> 来源：实车实验 Run 4~10，每一条都有日志可查。按症状→根因→解法组织。

---

## 1. 轨迹漂移

> 轨迹漂移的完整实验数据分析见 `reports/调参极限分析报告.md`。
>
> **一句话结论**：麦轮全向底盘存在轨迹漂移，`ABOTBASE=omni` 配置正确。优先排查：麦轮滚子打滑、轮子安装方向、里程计标定、轮距/轴距参数。

---

## 2. 导航与定位

### 2.1 AMCL 初始位姿错误导致粒子发散

**症状**：小车启动后 AMCL 长时间不收敛，位姿漂移严重。

**根因**：`initial_pose_x/y/a` 设为 (0,0,0)，但小车实际停放位置不是原点。AMCL 在错误位置初始化粒子，需要大量激光匹配才能修正。

**解法**：
- 在 `amcl.launch` 中设置正确的初始位姿（当前: x=0.6, y=-0.4, yaw=0）
- 或启动后用 `rostopic pub /initialpose` 手动设定
- 使用 `patrol_run.sh` 中的自动 initialpose 流程

### 2.2 move_base 耗时过长

**症状**：move_base 规划+执行一个目标点耗时远超预期，controller 反复调整。

**根因**：
- `controller_patience` 设为 15 秒，每次调整都等到超时才放弃
- TEB 的 `weight_max_vel_theta` 太小（=1），规划器倾向旋转而非平移

**解法**：
- `controller_patience` → 5 秒（`move_base_params.yaml`）
- `weight_max_vel_theta` → 10（`teb_omni_planner_params.yaml`）
- `oscillation_timeout` → 15（留足够时间恢复）

### 2.3 激光雷达扫描到车体导致避障异常

**症状**：小车原地打转或路径规划避开"自己"，因为激光扫到了车体边缘。

**根因**：`laser_filters` 的 `min_x` 不够大，车体部分被激光束覆盖。

**解法**：`x4_laserfilter.yaml` 中 `min_x` → -0.25（遮掉更多车体前方区域）。

### 2.4 Cartographer tracking_frame 配错

**症状**：建图时轨迹扭曲，点云无法对齐。

**根因**：`tracking_frame` 和 `published_frame` 设为 `laser`，应该是 `base_footprint`。Cartographer 需要知道机器人的坐标系而非传感器坐标系。

**解法**：`rplidar_lds.lua` 中改 `tracking_frame = "base_footprint"`，`published_frame = "base_footprint"`。

---

## 3. 硬件与通信

### 3.1 不同激光雷达型号波特率不同

**症状**：换了激光雷达后无数据输出。

**根因**：
- A1/A2: 115200
- A3: 256000
- C1: 460800

**解法**：在 `rplidar.launch` 中切换 `serial_baudrate`，默认已设为 460800（C1）。

### 3.2 里程计线性标定不准

**症状**：走 1 米实际位移误差 > 2cm。

**根因**：轮径磨损、地面摩擦力差异导致编码器计数偏移。

**解法**：在 `bringup.launch` 中微调 `linear_scale`（当前: 1.014）。标定方法：让小车走 1 米，量测实际距离，`linear_scale *= 目标/实际`。

---

## 4. 环境与网络

### 4.1 双网段 IP 切换

**症状**：SSH 突然连不上，ping 不通。

**根因**：小车接在不同的 WiFi 下会有不同 IP：
- 网段 A: `192.168.36.46`
- 网段 B: `192.168.43.211`

**解法**：
- 两个都 ping 一下，哪个通用哪个
- 记入 `README.md`（硬件详情章节）
- 连上车后 `hostname -I` 确认当前 IP

### 4.2 ROS_MASTER_URI 未设置

**症状**：本地笔记本无法 `rostopic list` 看小车话题。

**根因**：ROS 多机通信需要设置 `ROS_MASTER_URI=http://小车IP:11311`。

**解法**：笔记本端执行 `export ROS_MASTER_URI=http://192.168.43.211:11311`，并确保 `ROS_IP` 设为本机 IP。

---

## 5. 编码坑

### 5.1 Windows ↔ Linux 文件乱码

**症状**：YAML 文件中文注释在 Windows 上显示乱码。

**根因**：车上文件用 GBK 编码，Windows 本地默认也是 GBK，但编辑器可能用 UTF-8 打开。

**解法**：所有文件统一用 UTF-8 编码。`iconv -f gbk -t utf-8` 转换。

---

## 参考

- `logs/小车调节日志.txt` — 完整原始实验数据（Run 4~10）
- `reports/调参极限分析报告.md` — 数学推导与实验分析
- `README.md` — 硬件清单与当前状态
