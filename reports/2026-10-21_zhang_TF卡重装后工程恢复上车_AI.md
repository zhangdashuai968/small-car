# 2026-10-21 TF卡重装后工程恢复上车（zhang + AI）

## 任务

TF 卡重装官方系统（详见 `WORKLOG_20261021_小车TF卡救援与系统重建.md`）后，
从 git 仓库恢复小车工程文件，让小车可以正常启动、比赛。

## 一、本机 git 仓库整理（恢复的正本来源）

| 事项 | 结果 |
|---|---|
| 本地 main 落后 origin/main 37 提交，且本地 30b1680 与远程 4944772 为同一改动双提交 | main 已 reset 对齐 origin/main（无内容损失，patch 等价） |
| 工作区遗留 2026-06-11 实车最后状态未提交（waypoints 12 点重标定、localize_checkpoint、AMCL 重定位、DWA 中心线参数；与调参日志 [12:42]/[13:08]/[13:29] 记录吻合，文件 mtime 6-11 13:09/13:30 佐证） | 补提交 **defc712** |
| 恢复套件入库：restore_car.sh、5 篇 10 月报告、AGENTS.md、.gitignore 排除 abot_arm_learning/ | **a719eb7** |
| 7bc05fa 给 abotPID.cfg 加中文注释缺 utf-8 声明，Python2 gencfg 编译失败（上车编译实测复现） | 补声明修复 **0b1fb1c** |

**未能 push**：gh token 失效（`gh auth login` 待重做）+ 本地对象库缺 64dfab58（挂 7bc05fa
侧链，`git fetch --refetch` 因 GitHub 传输中断未修复）。网络好时需：重试 refetch → 验证
`git cat-file -t 64dfab58` → push main（3 个新提交待推）。

## 二、恢复包与上车

- 方案沿革：git bundle(272M) 因本地对象缺失弃用 → **git archive 工作区快照**
  `car_src_20261021.tar.gz`（272M / 2286 文件 / main@a719eb7）。
- 小车换 WiFi 后定位：**10.66.73.46**（22 端口探测；本机 10.66.73.247）。
- rsync 上车 68s（~4MB/s）；包与说明存 `~/桌面/car_restore_20261021/`。

## 三、车上恢复执行（restore_car.sh）

1. 旧 `~/catkin_ws`（10-06 快照 + build/devel）整体备份为 `catkin_ws.bak-20261021-140355`
2. 解包新工作区到 `~/catkin_ws`（比赛图 house.pgm/yaml、vl_locate、6-11 实车 runner/waypoints 均在位）
3. 5 个 GPU 依赖包 CATKIN_IGNORE 保持（abot_deep_learning / abot_yolo_detection /
   abot_object_trtdetect / abot_traffic_trtdetect / ros_astra_camera）
4. `catkin_make -j1`：首跑在 abot_pid 编码错误 → 车上 sed 同步修复 → 重跑
   **成功：342 目标全部 Built、[100%]、零错误、无 OOM（约 35 分钟）**

## 三点五、编译后静态验证（全部通过）

- rospack：abot / vl_locate / riki_msgs（即 abot_msgs 目录，package.xml 包名）/ move_base 均可发现
- devel：abot_base_node 及各包消息生成产物在位
- `from riki_msgs.msg import Servo/Battery` OK（runner 运行时依赖）
- bringup.launch / localize.launch XML 解析 OK；waypoints.yaml 12 点解析 OK
- 注：abot_msgs 报"not found"属正常——目录名 abot_msgs、包名 riki_msgs

## 四、硬件与系统状态

- 系统：L4T 32.6.1（内核 4.9.253-tegra），ROS Melodic，4 核 3.9G，磁盘 42G 空闲
- 串口映射（ttyACM0/ttyUSB0 谁是底盘/雷达）**尚未实车验证**，udev 99-abot.rules 已含防抢口

## 五、遗留事项（按优先级）

1. 编译完成后：bringup 实车验收（急停可达前提下）——`bash -ic 'roslaunch abot bringup.launch'`
   → rosnode list / rostopic hz /scan /odom / tf_echo map base_footprint
2. 串口映射验证（裸读两口，谁流数据谁是底盘 rosserial；反了改 99-abot.rules 两行 SYMLINK）
3. push：gh 重新 auth + refetch 修对象库 → push main（defc712/a719eb7/0b1fb1c）
4. vl_locate：conda py39 环境重建 + DashScope API key 换新（旧 key 已作废）
5. 车上无 .git（tar 快照模式）：网络好时可重取完整历史；期间车上改动需手动回传本机

## 下一步 / 交接

- 编译结果见调参日志 [22:16] 条目后续；若 opencv_apps 之后报错，按日志 grep "Error " 排查
- 比赛 runner：`python2 ~/catkin_ws/scripts/move_base_waypoint_runner.py`（含 6-11 实车状态）
- git SHA：恢复基线 a719eb7 + 编码修复 0b1fb1c（车上 = a719eb7 + sed 等效修复）
