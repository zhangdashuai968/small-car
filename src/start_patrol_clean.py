#!/usr/bin/env python
# -*- coding: utf-8 -*-
import paramiko, time, os

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(os.getenv('CAR_HOST', '192.168.43.211'),
          username=os.getenv('CAR_USER', 'abot'),
          password=os.getenv('CAR_PASSWORD'),
          timeout=5)

# 1. initialpose with covariance
print("Sending initialpose...")
cmd = """bash -ic "rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {frame_id: map, stamp: now}, pose: {pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.07]}}'" """
i, o, e = c.exec_command(cmd)
time.sleep(2)
print("initialpose sent")

# 2. clear costmaps
print("Clearing costmaps...")
i, o, e = c.exec_command('bash -ic "rosservice call /move_base/clear_costmaps {}"')
time.sleep(2)
print("costmaps cleared")

# 3. start patrol
print("Starting patrol (1 loop, rest=3s)...")
i, o, e = c.exec_command('bash -ic "rosrun abot abot_patrol_nav.py _keep_patrol:=false _patrol_loop:=1 _rest_time:=3"')
time.sleep(60)
out = o.read().decode()
print(out[-1500:])

c.close()
