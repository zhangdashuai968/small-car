# 自动建图和保存功能使用指南

本指南介绍如何使用自动建图和保存功能，在闭合空间内自动创建地图并保存。

## 功能概述

自动建图和保存功能包含以下组件：

1. `auto_map_save.py` - 核心Python脚本，负责启动SLAM建图、等待指定时间后停止建图并保存地图
2. `run_auto_map.sh` - 启动脚本，简化运行过程
3. `auto_map_save.launch` - ROS launch文件，用于启动自动建图节点

## 使用方法

### 方法1：使用Launch文件（推荐）

```bash
roslaunch abot auto_map_save.launch map_save_time:=120 map_name:=my_map
```

参数说明：
- `map_save_time`：建图时间（秒），默认120秒
- `map_name`：保存的地图名称，默认为auto_map

### 方法2：使用启动脚本

```bash
chmod +x ~/catkin_ws/src/abot_project/abot/scripts/run_auto_map.sh
~/catkin_ws/src/abot_project/abot/scripts/run_auto_map.sh 120 my_map
```

### 方法3：直接运行Python脚本

```bash
rosrun abot auto_map_save.py _map_save_time:=120 _map_name:=my_map
```

## 工作流程

1. 启动SLAM建图（使用lidar_slam.launch中的配置）
2. 机器人在环境中移动进行探索建图
3. 等待指定时间后自动停止SLAM
4. 使用map_server的map_saver保存地图
5. 地图文件保存到maps目录中

## 注意事项

1. 确保机器人在闭合空间内，以便获得完整地图
2. 建图时间应根据环境大小适当调整
3. 确保ROS环境已正确设置
4. 机器人需要配备激光雷达以进行SLAM建图

## 故障排除

1. 如果出现路径错误，请检查launch文件路径是否正确
2. 如果地图保存失败，请检查是否有写入maps目录的权限
3. 如果SLAM无法启动，请确保相关依赖包已安装