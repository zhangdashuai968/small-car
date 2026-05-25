#!/usr/bin/env python
#coding=utf-8

import rospy

import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist


class MultiControl():
    def __init__(self):

        master_robot = rospy.get_param('~master_robot', 'riki1')
        slave_robot = rospy.get_param('~slave_robot', 'riki2')

        #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
        self.vel_pub = rospy.Publisher(slave_robot + '/cmd_vel', Twist, queue_size=1)
        self.vel_sub = rospy.Subscriber(master_robot + '/cmd_vel', Twist, self.cbGetVel, queue_size=1)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            
            rate.sleep()


    def cbGetVel(self, vel_msg):
        vel = Twist()
        vel.linear.x = vel_msg.linear.x
        vel.linear.y = vel_msg.linear.y
        vel.angular.z = vel_msg.angular.z
        self.vel_pub.publish(vel)




if __name__ == '__main__':
    rospy.init_node('multi_control')
    node = MultiControl()
    rospy.spin()

        
