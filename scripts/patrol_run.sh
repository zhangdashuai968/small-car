#!/bin/bash
# 1. xy_goal_tolerance
rosparam set /move_base/TebLocalPlannerROS/xy_goal_tolerance 0.2
echo "xy_goal_tolerance set to 0.2"

# 2. initialpose with covariance (not zero)
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
  '{header: {frame_id: map, stamp: now}, pose: {pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.07]}}'
echo "initialpose sent, waiting 5s for AMCL..."
sleep 5

# 3. start patrol
echo "Starting patrol (1 loop, rest=3s)..."
rosrun abot abot_patrol_nav.py _keep_patrol:=false _patrol_loop:=1 _rest_time:=3
