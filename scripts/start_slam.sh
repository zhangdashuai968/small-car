#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ABOTMODEL=x4 CMM_MODE=rosserial ABOTBASE=omni ABOTIMU=mpu6050 ABOTLIDAR=rplidar
pkill -f "[l]ocalize.launch" 2>/dev/null; pkill -f "[a]mcl" 2>/dev/null; pkill -f "[m]ap_server" 2>/dev/null; sleep 2
pgrep -f "cartographer_slam" >/dev/null && echo SLAM-ALREADY-UP || { setsid nohup roslaunch abot cartographer_slam.launch > /tmp/enc_test/slam.log 2>&1 < /dev/null & echo SLAM-STARTED; }
sleep 12
echo ===VERIFY===
timeout 5 rostopic hz /scan 2>&1 | head -2
timeout 5 rostopic hz /odom 2>&1 | head -2
timeout 8 rosnode list 2>/dev/null | grep -cE "cartographer"
