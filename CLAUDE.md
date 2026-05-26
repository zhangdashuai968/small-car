# CLAUDE.md — ABOT M1 ARM 小车真机工作区

## 定位

**这个仓库只做一件事：真机实战。不做理论学习、不存学习笔记。**

| | 装（实战） | 不装（学习） |
|---|---|---|
| ✅ | ROS 源码、launch 文件、参数（`src/`） | 学习路线、实验 spec |
| ✅ | 运行脚本、航线、地图（`scripts/`） | 学习日志、学习笔记 |
| ✅ | 真机调试日志（`logs/`） | 进度追踪表（PROGRESS.md） |
| ✅ | 真机随手记录（`小车调节日志.txt`） | 速查表（cheatsheets） |
| ✅ | 踩坑手册（`TROUBLESHOOTING.md`） | |
| ✅ | 硬件清单（`真机信息.md`） | |
| ✅ | 会话报告（`reports/`） | |
| ✅ | 自动同步脚本（`auto-sync.sh`） | |

> **一句话**：这个仓库是"兵器库 + 航海日志"。学习资料全在 [abot_arm_learning](https://github.com/zhangdashuai968/abot_arm_learning)。

## 身份认知

- 这个仓库的代码**跑在 Jetson Nano 小车上**（`192.168.36.46`，用户 `abot`）
- 队友通过 SSH 连上小车操作，auto-sync 脚本自动把代码变更推到 GitHub
- 你的角色：给队友提供可执行的命令和建议，由队友在真机上执行
- 你不能直接操作真机
- 协作规范：参考 [abot_arm_learning/WORKFLOW.md](https://github.com/zhangdashuai968/abot_arm_learning/blob/main/WORKFLOW.md)

## Git 纪律（必须遵守）

- **会话开始时**：提醒队友 `git pull` 同步最新代码
- **每完成一个有意义的改动**：提醒队友 commit + push
  - Commit message 格式：`[EXX] 简短描述`（如 `[E23] Cartographer 子图数量改为 2`）
  - 不涉及实验的改动：`fix: 描述` / `chore: 描述`
- **会话结束前**：提醒队友最终 git push，确保所有改动已同步
- `scripts/auto-sync.sh` 每 30 秒自动提交，`sync-once.sh` 由 Windows 任务计划每 5 分钟触发
- **禁止**修改 `scripts/auto-sync.sh` 和 `scripts/sync-once.sh` 的同步逻辑
- **禁止** force push 到 main

## 日志纪律（必须遵守）

- 调试过程中的关键操作、命令输出、异常信息，**实时追加**到 `小车调节日志.txt`
- 格式：`[HH:MM] 内容描述`
- 如果调试涉及具体实验，同时写结构化日志到 `logs/YYYY-MM-DD_实验编号_简述.md`（模板：`logs/模板_调试日志.md`）
- 学习笔记不要放在这里——去 abot_arm_learning 的 `notes/` 写

## 安全规则（绝对不可违反）

- **密码**：`ssh-car.py` 中已有硬编码密码，新代码**绝不**再硬编码。用环境变量或配置文件读取
- **参数修改前先备份**：改 launch/param 文件前，记录原始值
- **运动控制前确认急停**：给出 `/cmd_vel` 或 move_base 命令前，提醒队友确认急停按钮可达
- **机械臂回 home 再断电**：每次会话结束前提醒：先归位再 Ctrl+C
- **底盘差速轮限制**：`twist.linear.y` 对差速轮底盘无效（详见 `调参极限分析报告.md`），使用"旋转→直行→旋转"替代侧移

## 报告要求（每次会话结束必须产出）

在 `reports/` 目录创建两份报告：

1. **人工报告**（`reports/YYYY-MM-DD_执行人_简述.md`）
   - 队友自己填写，使用 `reports/模板_人工报告.md` 模板
   - 5 分钟填完：做了什么、遇到什么问题、小车当前状态

2. **AI 报告**（`reports/YYYY-MM-DD_执行人_简述_AI.md`）
   - 你（AI）生成初稿，使用 `reports/模板_AI报告.md` 模板
   - 队友确认后提交
   - YAML 元数据块必须准确（git commit SHA、参数变更、硬件状态）

## 代码风格

- Python 脚本：ROS 节点用 `rospy`，工具脚本无框架要求
- YAML 配置：顶部注释参数含义和单位
- Launch 文件：顶部注释启动哪些节点和依赖关系
- 函数/关键逻辑用中文注释说明意图

## 会话启动检查清单

每次新会话开始，提醒队友：

- [ ] `git pull` 拉取最新代码
- [ ] 确认小车电池电压 ≥ 11.5V
- [ ] 确认 ROS Master 可达（`rostopic list` 有输出）
- [ ] 如做运动测试，确认急停按钮位置
- [ ] 读取最近的 `reports/*_AI.md` 了解上次状态

## 交叉引用

- 学习仓库：[abot_arm_learning](https://github.com/zhangdashuai968/abot_arm_learning)
  - 实验 spec：`parallel/` 目录
  - 实验→源码映射：`small-car-实验映射表.md`
  - 遇到不确定的参数含义，先去查对应的实验 spec
- 早报系统：[morning-newspaper](https://github.com/zhangdashuai968/morning-newspaper)
- 硬件参考：`真机信息.md`（硬件清单和已知 bug）
- 调参分析：`调参极限分析报告.md`（差速轮轨迹漂移根因分析）
- 踩坑手册：`TROUBLESHOOTING.md`
