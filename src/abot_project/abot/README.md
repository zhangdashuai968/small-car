# ABot机器人自动建图包

本包提供了ABot机器人在闭合空间内自动建图并保存的功能。

## 功能特性

- 自动启动SLAM建图
- 在指定时间后自动停止建图
- 自动保存生成的地图
- 支持参数配置（建图时间、地图名称等）

## 组成部分

### Python脚本
- `auto_map_save.py` - 核心自动建图和保存脚本

### Launch文件
- `auto_map_save.launch` - 启动自动建图节点的launch文件

### Shell脚本
- `run_auto_map.sh` - 便捷启动脚本
- `start_auto_mapping.sh` - 快速启动脚本

### 文档
- `docs/auto_map_guide.md` - 详细使用指南

## 依赖项

- ROS Melodic (Ubuntu 18.04)
- SLAM相关包 (gmapping, cartographer等)
- map_server包
- 机器人硬件：激光雷达、Mecanum轮底盘

## 使用方法

### 方法1：使用Launch文件（推荐）

```bash
roslaunch abot auto_map_save.launch map_save_time:=120 map_name:=my_map
```

### 方法2：使用启动脚本

```bash
./scripts/start_auto_mapping.sh
```

### 方法3：直接运行Python脚本

```bash
rosrun abot auto_map_save.py _map_save_time:=120 _map_name:=my_map
```

## 参数说明

- `map_save_time`：建图时间（秒），默认120秒
- `map_name`：保存的地图名称，默认为auto_map
- `map_directory`：地图保存目录，默认为包内的maps目录

## 地图保存位置

生成的地图文件将保存在`maps`目录中，包含`.pgm`（图像）和`.yaml`（配置）文件。

## 注意事项

1. 确保机器人在闭合空间内，以便获得完整地图
2. 建图时间应根据环境大小适当调整
3. 确保机器人配备了正常工作的激光雷达
4. 确保ROS环境已正确设置

## 故障排除

1. 如果出现路径错误，请检查launch文件路径是否正确
2. 如果地图保存失败，请检查是否有写入maps目录的权限
3. 如果SLAM无法启动，请确保相关依赖包已安装

