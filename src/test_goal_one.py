#!/usr/bin/env python
import paramiko, time

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.43.211', username='abot', password='123456', timeout=5)

# Send goal to point one
print("Sending goal to point one (1.168, 1.096)...")
cmd = """bash -ic "rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{header: {frame_id: map, stamp: now}, pose: {position: {x: 1.168, y: 1.096, z: 0}, orientation: {x: 0, y: 0, z: -0.047, w: 0.999}}}'" """
i, o, e = c.exec_command(cmd)
time.sleep(2)
print("Goal sent, monitoring 30s...")

# Monitor position every 5 seconds
for i in range(6):
    time.sleep(5)
    i2, o2, e2 = c.exec_command('bash -ic "rostopic echo -n 1 /amcl_pose"')
    out = o2.read().decode()
    pos = [l.strip() for l in out.split('\n') if 'x:' in l and 'z' not in l]
    print(f"[{(i+1)*5}s]", pos)

# Check move_base logs
print("\n=== Recent move_base logs ===")
i3, o3, e3 = c.exec_command('tail -300 /home/abot/.ros/log/690a422a-5717-11f1-927e-b46bfc324c95/rosout.log')
lines = o3.read().decode().split('\n')
keywords = ['TebLocal', 'recovery', 'Clear', 'Rotate', 'GOAL', 'oscillat', 'feasib', 'Going', 'Goal', 'succeed', 'fail', 'Timed']
for l in lines:
    if any(k in l for k in keywords):
        # Extract just the message part
        parts = l.split('] ')
        if len(parts) > 1:
            msg = parts[-1][:120]
            ts = l[:19]
            print(f"  {ts}: {msg}")

c.close()
