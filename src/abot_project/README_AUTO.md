# 一键启动导航抓取任务

## 使用方法

### 一键启动（推荐）
```bash
./scripts/one_click_start.sh
```

### 方法2：使用launch文件
```bash
# 手动启动各个服务（按顺序）
roslaunch abot bringup.launch
roslaunch abot navigate.launch  
conda activate 39 && roslaunch vl_locate vl_locate.launch
roslaunch ZachLab_grasp grasp.launch

# 运行自动化任务
roslaunch abot_project auto_navigation_grasp.launch
```

## 任务流程
1. 导航到点1 → 抓取蓝方块
2. 导航到点2 → 放置到图片中心点
3. 导航到点3 → 抓取红方块
4. 导航到点4 → 放置到图片中心点
5. 导航到点5 → 抓取绿方块
6. 导航到点6 → 放置到图片中心点
7. 导航到点7 → 任务完成

## 修改导航点
编辑 `scripts/auto_navigation_grasp.py` 中的 `waypoints` 数组来修改导航点坐标。