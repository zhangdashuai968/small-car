#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

class ProgressReporter:
    def __init__(self):
        rospy.init_node('progress_reporter')
        self.pub = rospy.Publisher('/cargo_handling_status', String, queue_size=10)
        self.rate = rospy.Rate(0.5)  # 2秒一次
        self.step = "初始化..."
        
        # 订阅步骤更新话题
        rospy.Subscriber('/update_step', String, self.update_step)
        
    def update_step(self, msg):
        self.step = msg.data
        
    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("当前进行步骤: {}".format(self.step))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        reporter = ProgressReporter()
        reporter.run()
    except rospy.ROSInterruptException:
        pass