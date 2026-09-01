# 会话报告（AI 版）

> 此报告由 AI 生成，人类队友确认后提交
> 下次会话的 AI 助手会读取此文件以恢复上下文
> 文件名：`YYYY-MM-DD_执行人_简述_AI.md`

---

## 元数据（机器解析用）

```yaml
session:
  date: "YYYY-MM-DD"
  executor: "姓名"
  experiment: "E00"          # 不涉及实验填 NONE
  duration_minutes: 0
  success: true              # true / false / partial

git:
  commit_before: "abc1234"
  commit_after: "def5678"
  files_changed:
    - path: "path/to/file"
      change_type: "modified"
      description: "改了什么"

vehicle_state:
  bringup_ok: true
  navigation_ok: true
  slam_ok: true
  arm_ok: true
  battery_voltage: 0.0
```

## 执行的操作序列

```bash
# 按顺序记录实际执行的关键命令
# 1. roslaunch abot bringup.launch
# 2. roslaunch abot navigate.launch
# 3. ...
```

## 参数变更

| 文件 | 参数名 | 旧值 | 新值 | 原因 |
|------|--------|------|------|------|
| | | | | |

## 异常记录

### 异常 1

- **症状**：
- **排查路径**：1 → 2 → 3
- **根因**：
- **修复方式**：

## 传感器/硬件状态

| 设备 | 状态 | 备注 |
|------|------|------|
| RPLIDAR 激光雷达 | OK | |
| Astra 深度相机 | OK | |
| IMU (MPU6050) | OK | |
| 机械臂 (ROCR6) | OK | |
| 底盘电机 (Zeus S2) | OK | |

状态取值：OK / DEGRADED（降级可用）/ FAIL（不可用）

## 未解决的问题

- [ ] 问题描述
- [ ]

## 下一次会话建议

（AI 对下一次会话的优先建议：先处理什么、注意什么风险）

---

> 关联：[人工报告](./YYYY-MM-DD_执行人_简述.md) | [调试日志模板](../logs/模板_调试日志.md) | [协作工作流](https://github.com/zhangdashuai968/abot_arm_learning/blob/master/docs/WORKFLOW.md)
