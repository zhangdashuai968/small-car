#!/bin/bash
# boot_all.sh - 一键启动: 底盘+定位+视觉 (放~/持久,重启不丢)
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ABOTMODEL=x4 CMM_MODE=rosserial ABOTBASE=omni ABOTIMU=mpu6050 ABOTLIDAR=rplidar
mkdir -p /tmp/enc_test
pgrep -f roscore >/dev/null || { setsid nohup roscore > /tmp/enc_test/roscore.log 2>&1 < /dev/null & sleep 4; }
pgrep -f "roslaunch abot bringup" >/dev/null || { setsid nohup roslaunch abot bringup.launch > /tmp/enc_test/bringup.log 2>&1 < /dev/null & sleep 10; }
pgrep -f "roslaunch abot localize" >/dev/null || { setsid nohup roslaunch abot localize.launch map_file:=/home/abot/catkin_ws/src/abot_project/abot/maps/house.yaml > /tmp/enc_test/localize.log 2>&1 < /dev/null & sleep 12; }
pgrep -f "vl_locate.launch" >/dev/null || { setsid nohup roslaunch vl_locate vl_locate.launch > /tmp/enc_test/vl_locate.log 2>&1 < /dev/null & sleep 12; }
echo ===VERIFY===
timeout 5 rostopic hz /odom 2>&1 | head -2
timeout 5 rostopic hz /new_topic 2>&1 | head -2
