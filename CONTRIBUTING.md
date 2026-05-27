# 协作指南 CONTRIBUTING

> 团队成员 + Claude Code 共用的贡献约定。改任何东西前先读这页。
> 仓库定位见 [`CLAUDE.md`](CLAUDE.md)，比赛硬约束见 [`比赛规则.md`](比赛规则.md)。

---

## 1. 平台事实（别再搞错）

| 项 | 值 |
|----|----|
| 计算平台 | NVIDIA Jetson Nano（Tegra X1，L4T R32.6.1） |
| 系统 / ROS | Ubuntu 18.04 / **ROS Melodic** |
| 车上 Python | **2.7**（ROS 节点）；vl_locate 用 conda 环境 `39` |
| 开发机 | Windows + PowerShell 7；工具脚本用 Python 3 |
| SSH | `abot@192.168.36.46` 或 `192.168.43.211` |

---

## 2. Git 流程

```bash
git pull --rebase                       # 开工先拉
# ... 改动 ...
git add <具体文件>                       # 不用 git add -A 误带文件
git commit -m "前缀: 描述"
git push                                # 撞车了见下方"撞车处理"
```

- **提交信息前缀**：`feat:` 新功能 / `fix:` 修 bug / `docs:` 文档 / `chore:` 杂务 / 实验改动用 `[EXX] 简述`。
- **不要** force-push `main`；**不要** 改 global git config；**不要** 跳过 hook。
- 新增可执行脚本后设执行位：`git update-index --chmod=+x <path>`（Windows 不带 exec bit，否则车上 `rosrun` 要手动 chmod）。

### 网络代理（本机 git 直连被墙）
```bash
git -c http.proxy=http://127.0.0.1:2080 -c https.proxy=http://127.0.0.1:2080 push
git -c http.proxy=http://127.0.0.1:2080 -c https.proxy=http://127.0.0.1:2080 fetch
```

### 撞车处理（auto-sync / 多人同推）
`scripts/auto-sync.sh` 每 30s 自动提交、`sync-once` 由 Windows 定时触发，所以 push 经常被拒：
```bash
git -c http.proxy=http://127.0.0.1:2080 fetch origin
git rebase origin/main          # 几乎不冲突(各改各文件); 冲突就解
git -c http.proxy=http://127.0.0.1:2080 push origin main
```

---

## 3. 编码 / 换行（已自动化，别手动转）

- 仓库根有 [`.gitattributes`](.gitattributes)（统一 LF）和 [`.editorconfig`](.editorconfig)（UTF-8/缩进）。
- **所有文本文件一律 UTF-8 + LF**。历史上车上有 GBK 文件导致 Windows 乱码（见 `TROUBLESHOOTING.md` §5）。
- 别再手动 `iconv` 或在编辑器里转 CRLF——交给上面两个配置。

---

## 4. 脚本放哪（重要，目前散在多处）

| 目录 | 放什么 |
|------|--------|
| `src/abot_project/scripts/` | **新运动/导航脚本归处**（`goal_nav.py` / `ten_point_race.py` / `ten_point_odom_race.py` / `seven_point_test.py` / `auto_navigation_grasp.py` 已在此） |
| `scripts/`（顶层） | 主机侧工具（`ssh-car.py` / `auto-sync.sh` / `sync-once.*` / `patrol_run.sh`）+ 历史脚本（`circle_run.py` 等） |
| `bags/` | rosbag 录制；**注意**这里有一份 `auto_task_runner.py` 与 `scripts/` 版本已分叉 ⚠️ |

> ⚠️ **待解决**：`bags/auto_task_runner.py`(283 行) 与 `scripts/auto_task_runner.py`(260 行) 内容不同。改它前先确认哪份为准（车上跑的是 `~/bags/`），别盲改。
> 新脚本一律放 `src/abot_project/scripts/`，并复用 `auto_task_runner` 的锁航向 cmd_vel 原语（绕开 move_base，见 CLAUDE.md）。

---

## 5. 团队必读 HTML 怎么重新生成

`团队必读文档汇总.html` 是由 [`tools/gen_docs.py`](tools/gen_docs.py) 合并多份 md 生成的（入库，方便离线单文件阅读）。改了被收录的 md 后重新生成：

```bash
pip install markdown
python tools/gen_docs.py        # 在仓库任意位置都能跑, 输出到仓库根
```

要增删收录文档，改 `tools/gen_docs.py` 里的 `DOCS` 列表。

---

## 6. 真机验证流程

```bash
ssh abot@192.168.43.211         # 密码见团队约定
# 非交互 shell 不加载 .bashrc, ABOT* env 会空 -> 用 bash -ic 取 env:
bash -ic 'roslaunch abot bringup.launch'
# 编译:
cd ~/catkin_ws && source /opt/ros/melodic/setup.bash && catkin_make
```

- 发运动指令前确认急停可达；机械臂回 home 再断电。
- 改参数前备份原值；调试操作实时记 `logs/小车调节日志.txt`，整段实验记 `logs/YYYY-MM-DD_*.md`（模板 `logs/模板_调试日志.md`）。
