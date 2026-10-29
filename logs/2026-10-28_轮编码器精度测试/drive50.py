#!/usr/bin/env python
# -*- coding: ascii -*-
# drive50.py - closed-loop backward drive, 0.1 m/s cruise + 0.03 m/s creep to 50cm.
# Safety against firmware cmd_vel-hold bug (no watchdog, latches last velocity):
#   1. stop is ALWAYS a 20Hz zero STREAM for 3s (single latched msg gets lost)
#   2. hard displacement cap 0.70m, time cap 15s, odom-stale cap 1s -> abort + zero stream
#   3. SIGINT/SIGTERM handler publishes zero stream before exit
#   4. verifies stationary before reporting RESULT
import rospy, signal, sys, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CRUISE = 0.10   # m/s backward
CREEP = 0.03    # m/s final phase
TARGET = 0.50   # m displacement magnitude
CREEP_AT = 0.40 # switch to creep at this displacement
HARD_MAX = 0.70 # m, abort
T_MAX = 15.0    # s, abort
ZERO_HOLD = 3.0 # s zero stream

state = {'x0': None, 'y0': None, 'x': None, 'y': None, 't0': None, 'last_msg_t': None}

def odom_cb(msg):
    p = msg.pose.pose.position
    now = rospy.get_time()
    if state['t0'] is None:
        state['t0'] = now
        state['x0'], state['y0'] = p.x, p.y
    state['x'], state['y'] = p.x, p.y
    state['last_msg_t'] = now

def zero_stream(pub, seconds):
    z = Twist()
    end = rospy.get_time() + seconds
    while rospy.get_time() < end and not rospy.is_shutdown():
        pub.publish(z)
        rospy.sleep(0.05)

def main():
    rospy.init_node('drive50')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/raw_odom', Odometry, odom_cb)

    def on_sig(*a):
        zero_stream(pub, ZERO_HOLD)
        sys.exit(1)
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    t_start = rospy.get_time()
    while state['x'] is None and rospy.get_time() - t_start < 5.0 and not rospy.is_shutdown():
        rospy.sleep(0.05)
    if state['x'] is None:
        print('RESULT FAIL no_odom')
        zero_stream(pub, ZERO_HOLD)
        return

    rate = rospy.Rate(20)
    reason = 'unknown'
    while not rospy.is_shutdown():
        dx = state['x'] - state['x0']; dy = state['y'] - state['y0']
        d = math.hypot(dx, dy)
        el = rospy.get_time() - state['t0']
        if d >= HARD_MAX:
            reason = 'ABORT_HARD_MAX'; break
        if el > T_MAX:
            reason = 'ABORT_TIMEOUT'; break
        if rospy.get_time() - state['last_msg_t'] > 1.0:
            reason = 'ABORT_ODOM_STALE'; break
        if d >= TARGET:
            reason = 'REACHED'; break
        t = Twist()
        t.linear.x = -(CREEP if d >= CREEP_AT else CRUISE)
        pub.publish(t)
        rate.sleep()

    zero_stream(pub, ZERO_HOLD)

    vx, vy = state['x'], state['y']
    still = False
    t_end = rospy.get_time() + 4.0
    while rospy.get_time() < t_end:
        rospy.sleep(0.1)
        if abs(state['x'] - vx) < 0.001 and abs(state['y'] - vy) < 0.001:
            still = True
            break
        vx, vy = state['x'], state['y']

    dx = state['x'] - state['x0']; dy = state['y'] - state['y0']
    print('RESULT %s final_d=%.4f dx=%.4f dy=%.4f stationary=%s'
          % (reason, math.hypot(dx, dy), dx, dy, still))

if __name__ == '__main__':
    main()
