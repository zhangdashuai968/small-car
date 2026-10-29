#!/usr/bin/env python
# -*- coding: ascii -*-
# test_rot90b.py - hardened rotation test (RIGHT 90 deg).
# Fixes vs test_rot90.py after 2026-10-29 over-rotation incident (cmd 90 -> phys 178):
#   1. stop phase = CONTINUOUS zero stream for 8s while monitoring yaw (not 3s burst)
#   2. settle check includes YAW rate (old version checked x/y only and missed spinning)
#   3. still spinning after 8s -> 5s more zero stream + STOP_ALARM in result
#   4. drive-phase hard abort at -115 deg (overshoot guard)
import rospy, signal, sys, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TARGET = math.radians(-180.0)
CRUISE_WZ = -0.3
CREEP_WZ = -0.08
CREEP_AT = math.radians(-165.0)
OVERSHOOT_LIMIT = math.radians(-215.0)
T_MAX = 20.0
STOP_STREAM_SEC = 8.0
YAW_RATE_OK = math.radians(2.0)  # deg/s threshold for "stopped spinning"

state = {'x': None, 'y': None, 'yaw': None, 'last_t': None}

def yaw_of(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

def odom_cb(msg):
    p = msg.pose.pose.position
    state['x'], state['y'] = p.x, p.y
    state['yaw'] = yaw_of(msg.pose.pose.orientation)
    state['last_t'] = rospy.get_time()

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def zero_duration(pub, sec):
    z = Twist()
    end = rospy.get_time() + sec
    while rospy.get_time() < end and not rospy.is_shutdown():
        pub.publish(z); rospy.sleep(0.05)

def main():
    rospy.init_node('test_rot90b')
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
    x0, y0, yaw0 = state['x'], state['y'], state['yaw']

    rate = rospy.Rate(20)
    reason = 'unknown'
    while not rospy.is_shutdown():
        done = wrap(state['yaw'] - yaw0)
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

    # stop phase: continuous zero stream while monitoring yaw trajectory
    z = Twist()
    t_end = rospy.get_time() + STOP_STREAM_SEC
    samples = []
    peak = wrap(state['yaw'] - yaw0)
    while rospy.get_time() < t_end and not rospy.is_shutdown():
        pub.publish(z)
        d = wrap(state['yaw'] - yaw0)
        if d < peak: peak = d
        samples.append((rospy.get_time(), d))
        rospy.sleep(0.05)

    yaw_rate = 99.0
    if len(samples) > 21:
        dt = samples[-1][0] - samples[-21][0]
        dyaw = wrap(samples[-1][1] - samples[-21][1])
        yaw_rate = abs(dyaw) / max(dt, 1e-6)

    alarm = False
    if yaw_rate > YAW_RATE_OK:
        alarm = True
        rospy.logwarn('STILL SPINNING after zero stream, last-resort 5s more')
        t2 = rospy.get_time() + 5.0
        while rospy.get_time() < t2 and not rospy.is_shutdown():
            pub.publish(z); rospy.sleep(0.05)
        d = wrap(state['yaw'] - yaw0)
        if d < peak: peak = d

    final = wrap(state['yaw'] - yaw0)
    print('RESULT %s odom_final=%.2fdeg odom_peak=%.2fdeg drift_dx=%.4f drift_dy=%.4f yaw_rate_end=%.1fdeg/s %s'
          % (reason, math.degrees(final), math.degrees(peak),
             state['x']-x0, state['y']-y0, math.degrees(yaw_rate),
             'STOP_ALARM' if alarm else 'stopped-ok'))

if __name__ == '__main__':
    main()
