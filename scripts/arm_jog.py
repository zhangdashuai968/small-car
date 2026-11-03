#!/usr/bin/env python
# -*- coding: utf-8 -*-
# arm_jog.py - M1-S arm jog/calibration tool (no ROS dependency)
# HOME (verified 2026-11-01): x=45 y=-5 z=230 rotary=90, gripper open
# usage:
#   python arm_jog.py home
#   python arm_jog.py x y z [rotary]
#   python arm_jog.py nudge axis step   (axis: x/y/z/r)
#   python arm_jog.py grip kai|he|ban   (open/close/half)
import sys, json, serial

PORT, BAUD = '/dev/ttyACM0', 115200
GRIP_OPEN, GRIP_CLOSE, GRIP_HALF = 150, 700, 500
HOME = (25, 0, 240, 90)
state = {'x': 25, 'y': 0, 'z': 240, 'rotary': 90}
ser = serial.Serial(PORT, BAUD, timeout=1)

def send_pose(x, y, z, rotary=90, speed=800):
    ser.write(json.dumps({"RikibotxArmCmd": 1,
                          "Pose": {"x": x, "y": y, "z": z, "rotary": rotary},
                          "time": speed}))
    state.update({'x': x, 'y': y, 'z': z, 'rotary': rotary})
    print("-> pose x=%d y=%d z=%d rotary=%d" % (x, y, z, rotary))

def send_grip(step):
    ser.write(json.dumps({"RikibotxArmCmd": 2, "Gripper": step, "time": 800}))
    print("-> gripper step=%d" % step)

if __name__ == '__main__':
    a = sys.argv
    if len(a) < 2:
        print(__doc__); sys.exit(1)
    if a[1] == 'home':
        send_pose(*HOME); send_grip(GRIP_OPEN)
    elif a[1] == 'grip':
        _g = {'kai': GRIP_OPEN, 'he': GRIP_CLOSE, 'ban': GRIP_HALF}.get(a[2])
        send_grip(_g if _g is not None else int(a[2]))
    elif a[1] == 'nudge' and len(a) >= 4:
        ax, d = a[2].lower(), int(a[3])
        send_pose(state['x'] + (d if ax == 'x' else 0),
                  state['y'] + (d if ax == 'y' else 0),
                  state['z'] + (d if ax == 'z' else 0),
                  state['rotary'] + (d if ax == 'r' else 0))
    else:
        send_pose(int(a[1]), int(a[2]), int(a[3]), int(a[4]) if len(a) > 4 else 90)
