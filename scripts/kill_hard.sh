#!/bin/bash
# kill_hard.sh — 硬停: 零速流5秒 → 杀move_base → 杀navigate → 杀脚本
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
timeout 6 python /tmp/estop.py >/dev/null 2>&1
pkill -9 -f "[g]oto_station" 2>/dev/null
pkill -9 -f "[f]ull_route" 2>/dev/null
PID=$(pgrep -f "move_base" | head -1); [ -n "$PID" ] && kill -9 $PID
PID=$(pgrep -f "navigate.launch" | head -1); [ -n "$PID" ] && kill -9 $PID
timeout 4 python /tmp/estop.py >/dev/null 2>&1
echo "HARD-STOP DONE"
