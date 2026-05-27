# CLAUDE.md — 真机工作区

## 定位

本仓库 = 兵器库 + 航海日志。只存真机实战内容，不存学习资料。
理论学习去 [abot_arm_learning](https://github.com/zhangdashuai968/abot_arm_learning)。

- 存：ROS 源码/launch/参数（`src/`）、脚本/航线/地图（`scripts/`）、调试日志（`logs/`）、会话报告（`reports/`）、团队文档（`docs/`）
- 不存：学习路线、实验 spec、学习日志、进度追踪表、速查表

## 比赛目标（一切工程的约束来源）

最终目标 = 室内移动抓取搬运赛（详见 `docs/比赛规则.md`）。**碰围栏/挡板 或 30s 不动即立即判负**——因此「只沿 xy 轴走、不漂移、原地转、fail-fast」是硬约束，不是偏好，给方案时必须遵守。

## 身份

- 代码跑在 Jetson Nano（Tegra X1，Ubuntu 18.04 / ROS Melodic，Python 2.7），SSH `abot@192.168.36.46` 或 `192.168.43.211`
- 队友通过 SSH 操作，auto-sync 自动同步到 GitHub
- 你给可执行命令和建议，由队友在真机上执行

## 当前导航方案（2026-05-26 起，勿推翻）

- **绕开 move_base/TEB/DWA**：局部规划器不稳 + 贴挡板避障有判负风险。运动走 `scripts/auto_task_runner.py` 自闭环原语（读 `map→base_footprint` tf 直接发 `cmd_vel`）。
- 定位用 `localize.launch`（amcl，无 move_base），初值 (0,0,0)；建图与 amcl 互斥（都发 `map→odom`，永远二选一）。
- **不要再建议调 TEB/DWA 参数**——方案已定向，调参属于已废弃方向。
- 细节见 `logs/2026-05-26_SLAM建图与局部规划器解耦.md`。

## 目录地图

```
small-car/
├── src/abot_project/   ← 自有 ROS 包(abot 主包 + grasp/vision/speech 等)，可改
│   ├── abot/           ← 主包: launch/ param/ maps/ rviz/ scripts/ script/
│   └── abot_project/scripts/  ← 新运动脚本归处(goal_nav, ten_point_*, seven_point_test)
├── src/<其余>/         ← vendored 第三方包(robot_localization/lidar/depth_camera/
│                          opencv_apps/robot_arm/imu_* 等)，**勿改、勿删**
├── scripts/            ← 主机侧工具(ssh-car/sync/patrol_run) + 历史测试脚本
├── bags/               ← rosbag; ⚠️ 内有与 scripts/ 分叉的 auto_task_runner.py
├── maps/ + abot/maps/  ← 栅格地图(comp.yaml=比赛图, house/my1_map=历史)
├── logs/ reports/      ← 调试日志 / 会话报告(均有模板)
├── tools/gen_docs.py   ← 生成 docs/团队必读文档汇总.html
├── docs/               ← 团队文档(架构总览/比赛规则/启动命令速查/TROUBLESHOOTING/汇总HTML)
└── README · CLAUDE · CONTRIBUTING  ← 根目录三件套(门面/AI中枢/协作约定)
```

## 脚本索引（运动执行，均绕开 move_base）

| 脚本(`src/abot_project/scripts/`) | 用途 | 前置 | 位姿源 |
|------|------|------|--------|
| `goal_nav.py` | 手动导航：rviz 点目标 → A* 全局规划 → 逐段执行 | bringup+localize | `map→base`(amcl) |
| `ten_point_race.py` | 基础十点比赛(抓放, 去抓取点前过放置点) | bringup+localize+grasp | `map→base`(amcl) |
| `ten_point_odom_race.py` | 十点纯里程计**保底**(定位栈挂也能跑) | 仅 bringup | `odom→base`(EKF) |
| `seven_point_test.py` | 七点可达性测试(纯导航) | bringup+localize | `map→base`(amcl) |
| `auto_task_runner.py` | 多点航线+抓放(读 waypoints.yaml) | bringup+localize | `map→base`(amcl) |

> 命令速查见 `docs/启动命令速查.md`；新脚本复用 `auto_task_runner` 锁航向原语。

## 运行 & 验证

```bash
roslaunch abot bringup.launch        # 底盘+IMU+EKF(odom→base)
roslaunch abot localize.launch       # map_server+amcl(map→odom); 与建图互斥
# 改完确认:
rosnode list                         # 无残留 move_base/旧节点
rostopic hz /scan                    # 雷达有数据
rosrun tf tf_echo map base_footprint # map→base 有输出=amcl 收敛
```

## Git

- **会话开始**：提醒 `git pull`
- **有意义的改动**：提醒 commit + push
  - 实验改动：`[EXX] 简短描述`
  - 非实验改动：`fix: 描述` / `chore: 描述`
- **会话结束**：提醒最终 push
- `scripts/auto-sync.sh` 每 30s 自动提交，`scripts/sync-once.sh` 由 Windows 每 5min 触发
- 禁止修改同步脚本逻辑，禁止 force push main

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

1. **人工报告**：队友填 `reports/YYYY-MM-DD_执行人_简述.md`（模板 `reports/模板_人工报告.md`）
2. **AI报告**：你生成初稿 `reports/YYYY-MM-DD_执行人_简述_AI.md`（模板 `reports/模板_AI报告.md`），YAML 元数据必须准确（git SHA、参数变更、硬件状态）

## 代码风格

- Python：ROS 节点用 `rospy`，工具脚本无框架要求
- YAML：顶部注释参数含义和单位
- Launch：顶部注释启动节点和依赖
- 关键逻辑用中文注释说明意图

## 会话启动检查清单

- [ ] `git pull`
- [ ] 电池电压 ≥ 11.5V
- [ ] ROS Master 可达（`rostopic list` 有输出）
- [ ] 运动测试前确认急停按钮位置
- [ ] 读最近 `reports/*_AI.md` 了解上次状态

## 交叉引用

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
