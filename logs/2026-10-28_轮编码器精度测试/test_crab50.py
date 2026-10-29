#!/usr/bin/env python
# -*- coding: ascii -*-
# test_crab50.py - validates the 3 crab fixes in the minimal test chain (no map TF):
#   1) dy-only arrival criterion (phantom dx no longer steals the lateral budget)
#   2) forward feedforward CRAB_X_DRIFT (constant backward drift 2~3cm/50cm)
#   3) post-stop yaw realign to pre-crab yaw (right crab drags -6~-8 deg)
# Same safety frame as drive50.py: 20Hz zero stream + watchdogs + stationary verify.
import rospy, signal, sys, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CRUISE = 0.10    # m/s rightward (negative linear.y)
CREEP = 0.03
TARGET = 0.50    # m, dy-only criterion
CREEP_AT = 0.40
HARD_MAX = 0.80  # m dy cap (realign rotation may add tiny x/y wiggle)
T_MAX = 25.0     # s incl. realign
ZERO_HOLD = 3.0
CRAB_X_DRIFT = 0.05
YAW_TOL = 0.03
K_YAW = 1.5; V_ROT = 0.4; V_ROT_MIN = 0.1

state = {'x': None, 'y': None, 'yaw': None, 'last_t': None}

def yaw_of(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

def odom_cb(msg):
    p = msg.pose.pose.position
    state['x'], state['y'] = p.x, p.y
    state['yaw'] = yaw_of(msg.pose.pose.orientation)
    state['last_t'] = rospy.get_time()

def zero_stream(pub, sec):
    z = Twist()
    end = rospy.get_time() + sec
    while rospy.get_time() < end and not rospy.is_shutdown():
        pub.publish(z); rospy.sleep(0.05)

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def main():
    rospy.init_node('test_crab50')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/raw_odom', Odometry, odom_cb)
    def on_sig(*a):
        zero_stream(pub, ZERO_HOLD); sys.exit(1)
    signal.signal(signal.SIGINT, on_sig); signal.signal(signal.SIGTERM, on_sig)
    t0 = rospy.get_time()
    while state['x'] is None and rospy.get_time() - t0 < 5.0 and not rospy.is_shutdown():
        rospy.sleep(0.05)
    if state['x'] is None:
        print('RESULT FAIL no_odom'); zero_stream(pub, ZERO_HOLD); return
    x0, y0, yaw0 = state['x'], state['y'], state['yaw']

    rate = rospy.Rate(20)
    reason = 'unknown'
    while not rospy.is_shutdown():
        dy = y0 - state['y']          # rightward progress (odom y decreases)
        if dy >= TARGET:
            reason = 'REACHED'; break
        if dy >= HARD_MAX:
            reason = 'ABORT_HARD'; break
        if rospy.get_time() - state['last_t'] > 1.0:
            reason = 'ABORT_ODOM_STALE'; break
        if rospy.get_time() - t0 > T_MAX:
            reason = 'ABORT_TIMEOUT'; break
        sp = CREEP if dy >= CREEP_AT else CRUISE
        tw = Twist()
        tw.linear.y = -sp
        tw.linear.x = CRAB_X_DRIFT * sp   # forward feedforward (body frame)
        pub.publish(tw)
        rate.sleep()
    zero_stream(pub, ZERO_HOLD)

    dy_crab = y0 - state['y']
    dx_crab = state['x'] - x0
    yaw_drag = wrap(state['yaw'] - yaw0)

    realigned = 'within_tol'
    if abs(yaw_drag) > YAW_TOL:
        t1 = rospy.get_time()
        while not rospy.is_shutdown():
            e = wrap(yaw0 - state['yaw'])
            if abs(e) < YAW_TOL:
                realigned = 'ok'; break
            if rospy.get_time() - t1 > 10.0:
                realigned = 'TIMEOUT(%.3frad)' % e; break
            wz = max(-V_ROT, min(V_ROT, K_YAW * e))
            if abs(wz) < V_ROT_MIN:
                wz = math.copysign(V_ROT_MIN, wz)
            tw = Twist(); tw.angular.z = wz
            pub.publish(tw); rate.sleep()
        zero_stream(pub, ZERO_HOLD)

    lx, ly = state['x'], state['y']
    still = False
    te = rospy.get_time() + 4.0
    while rospy.get_time() < te:
        rospy.sleep(0.1)
        if abs(state['x']-lx) < 0.001 and abs(state['y']-ly) < 0.001:
            still = True; break
        lx, ly = state['x'], state['y']

    yaw_final = wrap(state['yaw'] - yaw0)
    print('RESULT %s dy_crab=%.4f dx_crab=%.4f yaw_drag=%+.1fdeg yaw_final=%+.1fdeg realign=%s stationary=%s'
          % (reason, dy_crab, dx_crab,
             math.degrees(yaw_drag), math.degrees(yaw_final), realigned, still))

if __name__ == '__main__':
    main()
