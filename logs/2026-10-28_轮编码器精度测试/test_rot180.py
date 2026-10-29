#!/usr/bin/env python
# -*- coding: ascii -*-
# test_rot180.py - rotation test with CONTINUOUS accumulated angle (wrap-immune).
# Lesson 2026-10-29: wrap() output lives in (-180,+180], target of exactly
# -180 deg unreachable with wrap()-based done; accumulate per-sample deltas.
import rospy, signal, sys, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TARGET = math.radians(-180.0)
CRUISE_WZ = -0.3
CREEP_WZ = -0.08
CREEP_AT = math.radians(-165.0)
OVERSHOOT_LIMIT = math.radians(-215.0)
T_MAX = 25.0
STOP_STREAM_SEC = 8.0
YAW_RATE_OK = math.radians(2.0)

state = {'x': None, 'y': None, 'yaw_last': None, 'accum': 0.0,
         'last_t': None, 'dx': 0.0, 'dy': 0.0}

def yaw_of(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def odom_cb(msg):
    p = msg.pose.pose.position
    y = yaw_of(msg.pose.pose.orientation)
    if state['yaw_last'] is not None:
        state['accum'] += wrap(y - state['yaw_last'])
    state['yaw_last'] = y
    if state['x'] is None:
        state['dx'] = p.x
        state['dy'] = p.y
    state['x'], state['y'] = p.x, p.y
    state['last_t'] = rospy.get_time()

def zero_duration(pub, sec):
    z = Twist()
    end = rospy.get_time() + sec
    while rospy.get_time() < end and not rospy.is_shutdown():
        pub.publish(z); rospy.sleep(0.05)

def main():
    rospy.init_node('test_rot180')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/raw_odom', Odometry, odom_cb)
    def on_sig(*a):
        zero_duration(pub, 8.0); sys.exit(1)
    signal.signal(signal.SIGINT, on_sig); signal.signal(signal.SIGTERM, on_sig)
    t0 = rospy.get_time()
    while state['x'] is None and rospy.get_time() - t0 < 5.0 and not rospy.is_shutdown():
        rospy.sleep(0.05)
    if state['x'] is None:
        print('RESULT FAIL no_odom'); zero_duration(pub, 8.0); return

    rate = rospy.Rate(20)
    reason = 'unknown'
    while not rospy.is_shutdown():
        done = state['accum']
        if done <= TARGET:
            reason = 'REACHED'; break
        if done <= OVERSHOOT_LIMIT:
            reason = 'ABORT_OVERSHOOT'; break
        if rospy.get_time() - state['last_t'] > 1.0:
            reason = 'ABORT_ODOM_STALE'; break
        if rospy.get_time() - t0 > T_MAX:
            reason = 'ABORT_TIMEOUT'; break
        wz = CREEP_WZ if done <= CREEP_AT else CRUISE_WZ
        tw = Twist(); tw.angular.z = wz
        pub.publish(tw)
        rate.sleep()

    z = Twist()
    t_end = rospy.get_time() + STOP_STREAM_SEC
    samples = []
    peak = state['accum']
    while rospy.get_time() < t_end and not rospy.is_shutdown():
        pub.publish(z)
        if state['accum'] < peak: peak = state['accum']
        samples.append((rospy.get_time(), state['accum']))
        rospy.sleep(0.05)

    yaw_rate = 99.0
    if len(samples) > 21:
        dt = samples[-1][0] - samples[-21][0]
        dacc = samples[-1][1] - samples[-21][1]
        yaw_rate = abs(dacc) / max(dt, 1e-6)

    alarm = False
    if yaw_rate > YAW_RATE_OK:
        alarm = True
        t2 = rospy.get_time() + 5.0
        while rospy.get_time() < t2 and not rospy.is_shutdown():
            pub.publish(z); rospy.sleep(0.05)
        if state['accum'] < peak: peak = state['accum']

    print('RESULT %s odom_final=%.2fdeg odom_peak=%.2fdeg drift_dx=%.4f drift_dy=%.4f yaw_rate_end=%.1fdeg/s %s'
          % (reason, math.degrees(state['accum']), math.degrees(peak),
             state['x']-state['dx'], state['y']-state['dy'], math.degrees(yaw_rate),
             'STOP_ALARM' if alarm else 'stopped-ok'))

if __name__ == '__main__':
    main()
