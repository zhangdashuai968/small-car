#!/usr/bin/env python
# -*- coding: ascii -*-
# test_rot90_multi.py v2 - multi-pass rotate validation, SIGNED gap control.
# v1 lesson: one-directional threshold (done<=TARGET) never un-does overshoot;
# must use signed error so pass N rotates the other way. Mirrors the runner's
# P-control logic (which was already correct) on a wrap-immune accumulator.
import rospy, signal, sys, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TARGET = math.radians(-90.0)
CRUISE = 0.3    # rad/s magnitude
CREEP = 0.08
FINE = 0.08     # 0.04 below static friction - car does not move (lesson 11-01)
CREEP_AT = math.radians(15.0)
FINE_AT = math.radians(5.0)
HARD_ABS = math.radians(215.0)
T_MAX = 30.0
STOP_STREAM_SEC = 8.0
YAW_TOL = math.radians(1.72)
PASSES = 3

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
        state['dx'] = p.x; state['dy'] = p.y
    state['x'], state['y'] = p.x, p.y
    state['last_t'] = rospy.get_time()

def zero_duration(pub, sec):
    z = Twist()
    end = rospy.get_time() + sec
    while rospy.get_time() < end and not rospy.is_shutdown():
        pub.publish(z); rospy.sleep(0.05)

def main():
    rospy.init_node('test_rot90_multi')
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
    used = 0
    for attempt in range(PASSES):
        used = attempt + 1
        while not rospy.is_shutdown():
            gap = TARGET - state['accum']  # + = need right(neg wz), - = overshoot(need left)
            if abs(gap) < YAW_TOL:
                break
            if abs(state['accum']) > HARD_ABS:
                rospy.logwarn('hard bound'); break
            if rospy.get_time() - state['last_t'] > 1.0:
                break
            if rospy.get_time() - t0 > T_MAX:
                break
            if abs(gap) > CREEP_AT:
                sp = CRUISE
            elif abs(gap) > FINE_AT:
                sp = CREEP
            else:
                sp = FINE
            tw = Twist()
            tw.angular.z = math.copysign(sp, gap)  # gap<0 at start -> right(neg wz); overshoot -> gap>0 -> left
            pub.publish(tw)
            rate.sleep()
        zero_duration(pub, STOP_STREAM_SEC)
        te = rospy.get_time() + 1.0
        while rospy.get_time() < te:
            rospy.sleep(0.05)
        if abs(TARGET - state['accum']) < YAW_TOL:
            break
        rospy.logwarn('pass %d residual %+.2f deg', attempt + 1, math.degrees(TARGET - state['accum']))

    err = math.degrees(state['accum'] - TARGET)
    ok = 'PASS' if abs(err) < math.degrees(YAW_TOL) else 'FAIL'
    print('RESULT %s passes=%d final_err=%+.2fdeg total=%.2fdeg drift=(%.3f,%.3f)'
          % (ok, used, err, math.degrees(state['accum']),
             state['x']-state['dx'], state['y']-state['dy']))

if __name__ == '__main__':
    main()
