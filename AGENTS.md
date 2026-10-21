# small-car 项目约定

## 定位
本仓库是真机工作区：ROS 源码、launch、参数、地图、脚本、调试日志、会话报告和团队文档。学习路线、实验 spec、学习日志去 `abot_arm_learning`，不要混入本仓库。

`AGENTS.md` 是 Codex 主入口；`CLAUDE.md` 是历史规则来源和兼容参考。接手任务先读本文件，再按需读 `CLAUDE.md`、`README.md`、`CONTRIBUTING.md`、最近 `reports/*_AI.md` 和相关 `docs/`。

## 硬约束
- 最终目标是室内移动抓取搬运赛；碰围栏/挡板或 30s 不动即判负。
- 运动方案坚持“只沿 xy 轴走、不漂移、原地转、fail-fast”。
- 当前导航方案绕开 `move_base` / TEB / DWA，不再建议调局部规划器参数。
- 建图与定位互斥：gmapping 和 amcl 都发 `map->odom`，同一时间只能二选一。

## 环境
- 真机：Jetson Nano，Ubuntu 18.04，ROS Melodic，Python 2.7。
- 连接：`abot@192.168.36.46` 或 `abot@192.168.43.211`。
- Windows 侧主要做文件管理、审阅、脚本准备和 SSH 调度；真机命令默认由队友在车上执行。
- 在真机跑 ROS 命令时，用交互式 bash 或显式 source 环境，不假设非交互 shell 已加载 ROS。

## 目录边界
- `src/abot_project/` 是自有 ROS 包，可改。
- `src/<其余>/` 多为 vendored 第三方包，默认勿改、勿删。
- 新运动脚本优先放 `src/abot_project/scripts/`，复用 `auto_task_runner.py` 的锁航向原语。
- `logs/` 写调试日志，`reports/` 写会话报告，`docs/` 写团队文档。
- `bags/` 里可能有历史分叉脚本，不要默认当当前主版本。

## 运行与验证
常用真机启动：

```bash
roslaunch abot bringup.launch
roslaunch abot localize.launch
```

改动后最小验证优先看：

```bash
rosnode list
rostopic hz /scan
rosrun tf tf_echo map base_footprint
```

运动测试前确认急停按钮可达，机械臂断电前先回 home。

## 日志与交付
- 调试过程追加到 `logs/小车调节日志.txt`，格式 `[HH:MM] 内容`。
- 涉及具体实验时，同时写 `logs/YYYY-MM-DD_实验编号_简述.md`。
- 会话结束需要产出或更新 `reports/YYYY-MM-DD_执行人_简述_AI.md`，并记录 git SHA、参数变更、硬件状态和验证结果。
- 有意义改动提醒 commit + push；禁止 force push main，禁止改同步脚本逻辑。

