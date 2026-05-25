#!/bin/bash

# 自动建图和保存脚本
# 用法: ./run_auto_map.sh [建图时间(秒)] [地图名称]

# 检查ROS环境
if [ -z "$ROS_PACKAGE_PATH" ]; then
  echo "Error: ROS environment not set. Please source your ROS setup file first."
  exit 1
fi

# 检查是否已设置ROS核心
if ! rostopic list &> /dev/null; then
  echo "Starting ROS core..."
  roscore &
  sleep 3
fi

# 获取参数或使用默认值
MAP_SAVE_TIME=${1:-120}  # 默认建图120秒
MAP_NAME=${2:-auto_map}  # 默认地图名称为auto_map

# 启动自动建图和保存节点
echo "Starting automatic mapping for $MAP_SAVE_TIME seconds..."
echo "Map will be saved as $MAP_NAME"

rosrun abot auto_map_save.py _map_save_time:=$MAP_SAVE_TIME _map_name:=$MAP_NAME

# 等待并提示完成
sleep 2
echo "Automatic mapping process completed. Check the maps directory for the saved map."