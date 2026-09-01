# CLAUDE.md — 真机工作区

## 定位

本仓库 = 兵器库 + 航海日志。只存真机实战内容，不存学习资料。
理论学习去 [abot_arm_learning](https://github.com/zhangdashuai968/abot_arm_learning)。

- 存：ROS 源码/launch/参数（`src/`）、脚本/航线/地图（`scripts/`）、调试日志（`logs/`）、会话报告（`reports/`）、团队文档（`docs/`）
- 不存：学习路线、实验 spec、学习日志、进度追踪表、速查表

## 行为准则

### 先想后写
不确定时先问，不猜。先理解现有代码和配置，再动手改。

### 最少代码
只改必须改的。不做推测性功能。不引入不必要的抽象。

### 精准修改
只碰任务要求的文件。遵循现有代码风格。不"顺手优化"无关代码。

### 目标驱动
每个任务开始前定义成功标准。执行后必须验证。没验证 = 没完成。

### Risk 评估
非平凡修改必须输出至少 1 个具体失败模式 + 缓解措施。
涉及安全/硬件/不可逆参数的修改至少 2 个。

## 常用命令速查

```bash
# 底盘 + 定位
roslaunch abot bringup.launch       # 底盘+IMU+EKF
roslaunch abot navigate.launch      # map_server(house.yaml)+amcl+move_base(DWA) 主流程
roslaunch abot localize.launch      # map_server+amcl(无 move_base, 保底; 与建图互斥)

# 编译 & 检查
cd small_car/ && catkin_make        # 编译工作空间
rosnode list                        # 确认 move_base+amcl+map_server 在线
rostopic hz /scan                   # 雷达频率检查
rosrun tf tf_echo map base_footprint # 确认 amcl 收敛
```

## 比赛目标（一切工程的约束来源）

最终目标 = 室内移动抓取搬运赛（详见 `docs/比赛规则.md`）。**碰围栏/挡板 或 30s 不动即立即判负**——因此「只沿 xy 轴走、不漂移、原地转、fail-fast」是硬约束，不是偏好，给方案时必须遵守。

## 身份

- 代码跑在 Jetson Nano（Tegra X1，Ubuntu 18.04 / ROS Melodic，Python 2.7），SSH `abot@192.168.36.46` 或 `192.168.43.211`
- 仓库同步走「AI 会话收尾制」（见 Git 节），auto-sync 已弃用
- 两套上车通道并存：队友用 `scripts/ssh-car.py`（密码认证），AI 用免密密钥 SSH `ssh abot@<车IP>`（2026-09-01 起，密码不落文档）
- AI 可直接在真机执行，也可只给队友可执行命令。涉及运动/机械臂的命令，执行前队友必须在场且急停可达

## 当前导航方案（2026-06-01 起切回 move_base，06-07 起转 DWA）

- **已切回 move_base**：2026-05-26 曾绕开 move_base 走自闭环原语（`auto_task_runner.py` 直接发 `cmd_vel`）；06-01 切回，先 TEB（`teb_omni_planner_params.yaml`，见 commit `dc64d56`），06-07 起转 **DWA**（`dwa_omni_planner_params.yaml`：保守中心线参数 + 到点 yaw 对齐）。原语脚本降为保底。
- 主流程 `navigate.launch`（map_server 读 house.yaml + amcl 初值 (0.6,-0.4) + move_base(DWA)，规划器由 `NAV_PATH=dwa` 选择）；保底 `localize.launch`（amcl 初值 (0,0,0)，无 move_base）；建图与 amcl 互斥（都发 `map→odom`，永远二选一）。

## 目录地图

```
small-car/
├── src/abot_project/   ← 自有 ROS 包(abot 主包 + grasp/vision/speech 等)，可改
│   ├── abot/           ← 主包: launch/ param/ maps/ rviz/ scripts/ script/
│   └── abot_project/scripts/  ← 新运动脚本归处(goal_nav, ten_point_*, seven_point_test, auto_navigation_grasp)
├── src/<其余>/         ← vendored 第三方包(robot_localization/lidar/depth_camera/
│                          opencv_apps/robot_arm/imu_* 等)，**勿改、勿删**
├── scripts/            ← 主机侧工具(ssh-car/sync/patrol_run) + 历史测试脚本
├── bags/               ← rosbag; ⚠️ 内有与 scripts/ 分叉的 auto_task_runner.py
│       (地图在 abot/maps/: house.yaml=比赛图(navigate.launch 加载), my1_map=历史;
│        localize.launch 默认读 comp.yaml=车上建图存出, 仓库未跟踪)
├── logs/ reports/      ← 调试日志 / 会话报告(均有模板)
├── tools/gen_docs.py   ← 生成 docs/团队必读文档汇总.html
├── docs/               ← 团队文档(架构总览/比赛规则/启动命令速查/TROUBLESHOOTING/汇总HTML)
└── README · CLAUDE · CONTRIBUTING  ← 根目录三件套(门面/AI中枢/协作约定)
```

## 脚本索引（move_base 主流程 + 自闭环原语保底）

| 脚本(`src/abot_project/scripts/`) | 用途 | 前置 | 位姿源 |
|------|------|------|--------|
| `auto_navigation_grasp.py` | 10 点完整抓放主流程(内联航线坐标, 含 VLM + 机械臂握手) | bringup+navigate+vl_locate+grasp | `map→base`(amcl) |
| `goal_nav.py` | 手动导航：rviz 点目标 → A* 全局规划 → 逐段执行 | bringup+localize | `map→base`(amcl) |
| `ten_point_race.py` | 基础十点比赛(抓放, 去抓取点前过放置点) | bringup+localize+grasp | `map→base`(amcl) |
| `ten_point_odom_race.py` | 十点纯里程计**保底**(定位栈挂也能跑) | 仅 bringup | `odom→base`(EKF) |
| `seven_point_test.py` | 七点可达性测试(纯导航) | bringup+localize | `map→base`(amcl) |

> 顶层 `scripts/` 另有两个航线脚本：`move_base_waypoint_runner.py`（读 `waypoints.yaml` 发 move_base，`--nav-only` 跳过 VLM/机械臂）与 `auto_task_runner.py`（原语版多点抓放；⚠️ `bags/` 有分叉副本）。
> 命令速查见 `docs/启动命令速查.md`；新运动脚本一律放 `src/abot_project/scripts/`（主流程走 move_base，原语仅保底）。

## 运行 & 验证

```bash
roslaunch abot bringup.launch        # 底盘+IMU+EKF(odom→base)
roslaunch abot navigate.launch       # map_server(house.yaml)+amcl+move_base(DWA); 与建图互斥
# 改完确认:
rosnode list                         # move_base + amcl + map_server 均在线
rostopic hz /scan                    # 雷达有数据
rosrun tf tf_echo map base_footprint # map→base 有输出=amcl 收敛
```

## Git

- 仓库地址：[small-car](https://github.com/zhangdashuai968/small-car)
- **会话开始**：`git pull`（拉不动先查代理，见 CONTRIBUTING §2）
- **有意义的改动**：commit + push
  - 实验改动：`[EXX] 简短描述`
  - 非实验改动：`fix: 描述` / `feat:` / `docs:` / `chore: 描述`
- **会话结束 = AI 会话收尾制（2026-09-01 起）**：AI 负责——add 具体文件（不用 `add -A`）→ 分组 commit → push → 生成 AI 报告（`reports/`）→ 追加 `logs/小车调节日志.txt`
- ~~auto-sync.sh 每 30s 自动提交~~ **已弃用**（实测 3 个月仅成功 1 次，Windows 计划任务已删；脚本保留仅作历史，勿再启用）。禁 force push main 不变
- ⚠️ Windows 大小写：`claude.md` 重复跟踪已于 2026-09-01 移除（NTFS 不分大小写，双跟踪会互相覆盖），**禁止创建仅大小写不同的文件名**

## 日志

- 调试操作/命令/异常实时追加到 `logs/小车调节日志.txt`，格式 `[HH:MM] 内容`
- 涉及具体实验时同时写 `logs/YYYY-MM-DD_实验编号_简述.md`（模板：`logs/模板_调试日志.md`）
- 学习笔记去 abot_arm_learning 写

## 安全（不可违反）

- 密码：仅 `scripts/ssh-car.py` 可硬编码，新代码用环境变量或配置文件
- 改参数前先备份原始值
- 给 `/cmd_vel` 或 move_base 命令前，确认急停按钮可达
- 机械臂回 home 再断电
- 麦克纳姆轮底盘，`ABOTBASE=omni` 为正确配置（含 `tw.linear.y` 侧移能力）

## 会话报告（每次结束必须产出）

1. **AI 报告（主轨，每次必产出）**：AI 生成 `reports/YYYY-MM-DD_执行人_简述_AI.md`（模板 `reports/模板_AI报告.md`），YAML 元数据必须准确（git SHA、参数变更、硬件状态）
2. **人工速记（辅轨，2026-09-01 减负版）**：队友填 `reports/YYYY-MM-DD_执行人_简述.md`，5 行即可（模板 `reports/模板_人工报告.md`），重点写 AI 看不到的现场信息（异响、手感、环境变化）

## 代码风格

- Python：ROS 节点用 `rospy`，工具脚本无框架要求
- YAML：顶部注释参数含义和单位
- Launch：顶部注释启动节点和依赖
- 关键逻辑用中文注释说明意图

## 会话启动检查清单

- [ ] `git pull`（机制自检：核对下方「当前导航方案」日期 ≥ 最近一份 AI 报告日期——旧版中枢文档会误导）
- [ ] 电池电压 ≥ 11.5V
- [ ] ROS Master 可达（`rostopic list` 有输出）
- [ ] 运动测试前确认急停按钮位置
- [ ] 读最近 `reports/*_AI.md` 了解上次状态

## 交叉引用

- 本仓库：[small-car](https://github.com/zhangdashuai968/small-car)
- 学习仓库：[abot_arm_learning](https://github.com/zhangdashuai968/abot_arm_learning)（实验 spec 在 `parallel/`，映射表 `small-car-实验映射表.md`）
- 早报：[morning-newspaper](https://github.com/zhangdashuai968/morning-newspaper)
- 硬件：`README.md`
- 协作约定（git/编码/脚本放哪）：`CONTRIBUTING.md`
- 架构总览（tf树/节点/数据流）：`docs/ARCHITECTURE.md`
- 比赛规则：`docs/比赛规则.md`
- 命令速查：`docs/启动命令速查.md`
- 调参：`reports/调参极限分析报告.md`
- 踩坑：`docs/TROUBLESHOOTING.md`
- 文档汇总生成器：`tools/gen_docs.py`
