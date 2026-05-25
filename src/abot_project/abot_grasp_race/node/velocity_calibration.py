#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
速度校准节点 - 使用激光雷达校准线速度和角速度
"""

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class VelocityCalibrator:
    def __init__(self):
        # 校准参数
        self.test_linear_vel = rospy.get_param('~test_linear_vel', 0.2)
        self.test_angular_vel = rospy.get_param('~test_angular_vel', 0.5)
        self.test_duration = rospy.get_param('~test_duration', 5.0)
        
        # 订阅激光雷达和里程计
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 发布速度命令
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.laser_data = None
        self.odom_data = None
        
        rospy.loginfo("速度校准节点初始化完成")
        rospy.sleep(1.0)
    
    def scan_callback(self, msg):
        self.laser_data = msg
    
    def odom_callback(self, msg):
        self.odom_data = msg
    
    def get_front_distance(self):
        """获取前方距离"""
        if self.laser_data is None:
            return None
        
        ranges = self.laser_data.ranges
        total = len(ranges)
        front_ranges = []
        
        # 前方±15度
        for i in range(-15, 16):
            idx = i % total
            if ranges[idx] > 0.1 and ranges[idx] < 10.0:
                front_ranges.append(ranges[idx])
        
        return min(front_ranges) if front_ranges else None
    
    def get_odom_position(self):
        """获取里程计位置"""
        if self.odom_data is None:
            return 0.0, 0.0
        
        x = self.odom_data.pose.pose.position.x
        y = self.odom_data.pose.pose.position.y
        return x, y
    
    def get_odom_yaw(self):
        """获取里程计偏航角"""
        if self.odom_data is None:
            return 0.0
        
        orientation = self.odom_data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        return yaw
    
    def stop_robot(self):
        """停止机器人"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
    
    def calibrate_linear_velocity(self):
        """校准线速度"""
        rospy.loginfo("\n========== 线速度校准 ==========")
        rospy.loginfo("请将机器人放在距离墙壁2-3米的位置，面向墙壁")
        rospy.loginfo("按Enter开始测试...")
        raw_input()
        
        # 等待激光雷达数据
        while self.laser_data is None:
            rospy.sleep(0.1)
        
        # 记录初始距离
        start_distance = self.get_front_distance()
        if start_distance is None:
            rospy.logerr("无法获取前方距离，请检查激光雷达")
            return
        
        # 记录里程计初始位置
        x0, y0 = self.get_odom_position()
        
        rospy.loginfo("初始距离: %.3f 米", start_distance)
        rospy.loginfo("开始前进，速度: %.2f m/s，时长: %.1f 秒", 
                     self.test_linear_vel, self.test_duration)
        
        # 前进
        cmd = Twist()
        cmd.linear.x = self.test_linear_vel
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)
        
        while (rospy.Time.now() - start_time).to_sec() < self.test_duration:
            self.cmd_pub.publish(cmd)
            rate.sleep()
        
        self.stop_robot()
        rospy.sleep(0.5)
        
        # 记录结束距离
        end_distance = self.get_front_distance()
        x1, y1 = self.get_odom_position()
        
        if end_distance is None:
            rospy.logerr("无法获取结束距离")
            return
        
        # 计算实际移动距离
        laser_distance = start_distance - end_distance
        odom_distance = math.sqrt((x1-x0)**2 + (y1-y0)**2)
        
        rospy.loginfo("\n结束距离: %.3f 米", end_distance)
        rospy.loginfo("激光雷达测量移动距离: %.3f 米", laser_distance)
        rospy.loginfo("里程计测量移动距离: %.3f 米", odom_distance)
        
        # 计算校准系数
        if odom_distance > 0.01:
            linear_calibration = laser_distance / odom_distance
            rospy.loginfo("\n线速度校准系数: %.4f", linear_calibration)
            rospy.loginfo("建议修改参数: linear_scale = %.4f", linear_calibration)
        else:
            rospy.logerr("里程计数据异常")
    
    def calibrate_angular_velocity(self):
        """校准角速度"""
        rospy.loginfo("\n========== 角速度校准 ==========")
        rospy.loginfo("请将机器人放在开阔区域")
        rospy.loginfo("按Enter开始测试...")
        raw_input()
        
        # 等待激光雷达数据
        while self.laser_data is None:
            rospy.sleep(0.1)
        
        # 记录初始角度
        yaw_start = self.get_odom_yaw()
        
        rospy.loginfo("开始旋转，角速度: %.2f rad/s，时长: %.1f 秒", 
                     self.test_angular_vel, self.test_duration)
        
        # 旋转
        cmd = Twist()
        cmd.angular.z = self.test_angular_vel
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)
        
        while (rospy.Time.now() - start_time).to_sec() < self.test_duration:
            self.cmd_pub.publish(cmd)
            rate.sleep()
        
        self.stop_robot()
        rospy.sleep(0.5)
        
        # 记录结束角度
        yaw_end = self.get_odom_yaw()
        
        # 计算旋转角度
        odom_angle = yaw_end - yaw_start
        
        # 归一化到[-pi, pi]
        while odom_angle > math.pi:
            odom_angle -= 2 * math.pi
        while odom_angle < -math.pi:
            odom_angle += 2 * math.pi
        
        # 理论旋转角度
        expected_angle = self.test_angular_vel * self.test_duration
        
        rospy.loginfo("\n理论旋转角度: %.2f 度 (%.3f rad)", 
                     math.degrees(expected_angle), expected_angle)
        rospy.loginfo("里程计旋转角度: %.2f 度 (%.3f rad)", 
                     math.degrees(odom_angle), odom_angle)
        
        # 计算校准系数
        if abs(odom_angle) > 0.01:
            angular_calibration = expected_angle / odom_angle
            rospy.loginfo("\n角速度校准系数: %.4f", angular_calibration)
            rospy.loginfo("建议修改参数: angular_scale = %.4f", angular_calibration)
        else:
            rospy.logerr("里程计数据异常")
    
    def run(self):
        """运行校准流程"""
        rospy.loginfo("\n速度校准程序")
        rospy.loginfo("=" * 50)
        
        while not rospy.is_shutdown():
            rospy.loginfo("\n请选择校准项目:")
            rospy.loginfo("1. 线速度校准")
            rospy.loginfo("2. 角速度校准")
            rospy.loginfo("3. 全部校准")
            rospy.loginfo("4. 退出")
            
            choice = raw_input("请输入选项 (1-4): ")
            
            if choice == '1':
                self.calibrate_linear_velocity()
            elif choice == '2':
                self.calibrate_angular_velocity()
            elif choice == '3':
                self.calibrate_linear_velocity()
                rospy.sleep(2.0)
                self.calibrate_angular_velocity()
            elif choice == '4':
                rospy.loginfo("退出校准程序")
                break
            else:
                rospy.logwarn("无效选项")

def main():
    rospy.init_node('velocity_calibration')
    calibrator = VelocityCalibrator()
    calibrator.run()

if __name__ == '__main__':
    main()
