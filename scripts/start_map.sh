#!/bin/bash

# 杀死所有旧进程
pkill -9 ros 2>/dev/null || true
sleep 2

# 1. 启动底盘 → 等待10秒（让底盘完全初始化，发布坐标系）
gnome-terminal --title="底盘驱动" -- bash -c "
source ~/catkin_ws/devel/setup.bash
roslaunch abot bringup.launch
exec bash
"
sleep 10

# 3. 启动键盘控制
gnome-terminal --title="键盘控制" -- bash -c "
source ~/catkin_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
exec bash
"
sleep 3

# 4. 启动建图算法
gnome-terminal --title="建图节点" -- bash -c "
source ~/catkin_ws/devel/setup.bash
roslaunch abot cartographer_slam.launch
exec bash
"
sleep 5

# 5. 启动RViz
gnome-terminal --title="RViz可视化" -- bash -c "
rviz -d ~/catkin_ws/src/abot_project/abot/rviz/slam.rviz
exec bash
"
