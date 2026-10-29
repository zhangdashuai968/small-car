#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
mkdir -p /tmp/enc_test
setsid nohup roscore > /tmp/enc_test/roscore.log 2>&1 < /dev/null &
sleep 4
setsid nohup rosrun rosserial_python serial_node.py _port:=/dev/abotbase _baud:=115200 > /tmp/enc_test/serial.log 2>&1 < /dev/null &
sleep 3
setsid nohup rosrun abot abot_base_node _linear_scale:=1.0 > /tmp/enc_test/base.log 2>&1 < /dev/null &
sleep 2
echo ===NODES===
timeout 5 rosnode list
echo ===SCALE===
rosparam get /riki_base_node/linear_scale
echo ===HZ===
timeout 5 rostopic hz /raw_odom 2>&1 | head -2
echo ===POS===
timeout 4 rostopic echo -n1 /raw_odom/pose/pose/position
