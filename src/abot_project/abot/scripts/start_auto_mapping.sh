#!/bin/bash

# 快速启动自动建图脚本

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

# 启动自动建图和保存功能
echo "Starting automatic mapping and saving..."
echo "Robot will start mapping for 120 seconds, then save the map as 'auto_generated_map'"

roslaunch abot auto_map_save.launch map_save_time:=120 map_name:=auto_generated_map

# 等待并提示完成
sleep 2
echo "Automatic mapping process completed. Check the maps directory for the saved map."