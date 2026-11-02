#!/bin/bash
# start_grasp.sh - 启动ZachLab抓取节点(杀掉grasp_race防串口冲突)
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
pkill -9 -f "[g]rasp_race" 2>/dev/null; pkill -f "[g]g.py" 2>/dev/null; sleep 2
pgrep -f "ZachLab_grasp" >/dev/null && echo GRASP-ALREADY-UP || { setsid nohup roslaunch ZachLab_grasp grasp.launch > /tmp/enc_test/zachlab.log 2>&1 < /dev/null & echo STARTED; }
sleep 8
timeout 8 rosnode list 2>/dev/null | grep basic_xarm
timeout 4 rosparam get /basic_xarm_grasp/cali_3_mc 2>/dev/null | tr "\n" " "; echo
