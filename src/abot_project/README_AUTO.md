# 一键启动导航抓取任务

## 使用方法

### 一键启动（推荐）
```bash
cd ~/Desktop/small-car/src/abot_project/scripts && ./one_click_start.sh
```
（按优先级启动：`bringup -> localize`，`VL locate/Grasp` 后启动并降优先级，最后执行 `hardcoded_race.py`。路线是固定相对位移队列：走 x/y 米 → 执行抓放/路过 → 等待 → 下一段。）

> 已绑定 `cd`，可直接用完整路径执行：
> `~/Desktop/small-car/src/abot_project/scripts/one_click_start.sh`

### 绑定命令（不再手工 cd）
```bash
~/Desktop/small-car/src/abot_project/scripts/one_click_start.sh --help
```

纯导航 dry-run（不抓放）：
```bash
cd ~/Desktop/small-car/src/abot_project/scripts && ./one_click_start.sh --nav-only
```

只打印硬编码路线（不发 `/cmd_vel`）：
```bash
cd ~/Desktop/small-car/src/abot_project/scripts && ./one_click_start.sh --dry-run
```

### 小车远程桌面启动（推荐）
```bash
# 本地远程桌面上直接执行（任意目录可用）
bash ~/Desktop/small-car/src/abot_project/scripts/one_click_start.sh

# 若远端机器以 /home/abot/catkin_ws 作为工作区
bash -lc 'bash /home/abot/catkin_ws/src/abot_project/scripts/one_click_start.sh'
```
如果远程是 SSH 控制（例如从Windows执行）：
```bash
ssh abot@192.168.36.46 "bash -lc '/home/abot/catkin_ws/src/abot_project/scripts/one_click_start.sh'"
```

常用控制：
```bash
# 只检查状态（不启动）
bash ~/Desktop/small-car/src/abot_project/scripts/one_click_start.sh status
# 重跑任务脚本（不重启四个 launch）
bash ~/Desktop/small-car/src/abot_project/scripts/one_click_start.sh rerun
# 从第 6 段继续
bash ~/Desktop/small-car/src/abot_project/scripts/one_click_start.sh rerun --from-step 6
# 紧急刹停（不重启 launch）
bash ~/Desktop/small-car/src/abot_project/scripts/one_click_start.sh brake
```

### 方法2：使用launch文件
```bash
# 手动启动各个服务（按顺序）
roslaunch abot bringup.launch
roslaunch abot localize.launch
conda activate 39 && roslaunch vl_locate vl_locate.launch
roslaunch ZachLab_grasp grasp.launch

# 运行自动化任务
script="$(rospack find abot_project)/scripts/hardcoded_race.py"
python2 "${script}"
```

## 任务流程
1. 从 `(0,0)` 出发，到 `(-0.6,-1.8)`。
2. 到 `(-0.6,-3.0)`，执行视觉识别和机械臂抓取。
3. 回到 `(-0.6,-1.8)`，执行机械臂放置。
4. 到 `(-0.6,-0.6)`，再到 `(-1.8,-0.6)`。
5. 到 `(-1.8,-1.8)`，再到 `(-1.8,-3.0)`，执行视觉识别和机械臂抓取。
6. 回到 `(-1.8,-1.8)`，执行机械臂放置。
7. 到 `(-1.8,-0.6)`，再到 `(-3.0,-0.6)`。
8. 到 `(-3.0,-1.8)`，再到 `(-3.0,-3.0)`，执行视觉识别和机械臂抓取。
9. 回到 `(-3.0,-1.8)`，执行机械臂放置。
10. 到 `(-3.4,-3.4)`，关闭抓取并让舵机回 home。

## 修改硬编码距离
编辑 `src/abot_project/scripts/hardcoded_race.py` 顶部 `ROUTE`。`x` 正方向是前进，`y` 正方向是左移，单位 m。
