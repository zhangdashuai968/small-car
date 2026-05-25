#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
多点导航校准脚本

功能：
- 自动遍历所有导航点
- 记录实际到达位置与目标位置的偏差
- 生成校准后的参数文件

适用环境：
- Ubuntu 18.04 + ROS Melodic
- 麦克纳姆轮底盘机器人
- 配备激光雷达

作者：ROS开发团队
创建日期：2024年
版本：1.0
"""

import rospy
import actionlib
import json
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


class MultiPointNavigationCalibrator:
    """
    多点导航校准类
    
    负责自动遍历导航点、记录位置偏差并生成校准参数文件
    """
    def __init__(self):
        """
        初始化多点导航校准节点
        """
        rospy.init_node('multi_point_navigation_calibrator')
        
        # move_base动作客户端 - 用于导航控制
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        # TF监听器 - 用于获取机器人当前位置
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 8个导航点位配置（示例值，需根据实际地图调整）
        # 每个点位包含：坐标、朝向
        self.waypoints = [
            {'x': 0.4, 'y': 2.8, 'yaw': 0.0},
            {'x': 0.4, 'y': 1.6, 'yaw': 1.0},
            {'x': 1.6, 'y': 2.8, 'yaw': 2.0},
            {'x': 1.6, 'y': 1.6, 'yaw': 3.0},
            {'x': 2.8, 'y': 2.8, 'yaw': -1.0},
            {'x': 2.8, 'y': 1.6, 'yaw': -2.0},
            {'x': 3.2, 'y': 0.0, 'yaw': -3.0},
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        ]
        
        # 校准结果存储
        self.calibration_results = []
    
    def send_nav_goal(self, x, y, yaw):
        """
        发送导航目标到move_base
        
        参数：
        x (float): 目标点X坐标
        y (float): 目标点Y坐标
        yaw (float): 目标朝向（弧度）
        
        返回：
        int: 导航结果状态码
            - 3: 导航成功
            - 其他: 导航失败
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # 使用map坐标系
        goal.target_pose.header.stamp = rospy.Time.now()  # 当前时间戳
        
        # 设置目标位置
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0  # 地面高度
        
        # 简化：只设置yaw，四元数转换略
        # 注意：实际应用中需要将yaw转换为四元数
        goal.target_pose.pose.orientation.w = 1.0  # 默认朝向
        
        # 发送目标并等待结果
        result = self.move_base_client.send_goal_and_wait(goal)
        return result
    
    def get_current_position(self):
        """
        获取机器人当前位置
        
        返回：
        dict: 包含当前位置信息的字典，包括x, y, yaw
        """
        try:
            # 获取机器人在map坐标系下的位置
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
            
            # 提取位置信息
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # 简化：只提取yaw，四元数转换略
            # 注意：实际应用中需要将四元数转换为yaw
            yaw = 0.0  # 默认值
            
            return {'x': x, 'y': y, 'yaw': yaw}
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("无法获取机器人当前位置")
            return None
    
    def run(self):
        """
        主运行循环：遍历所有导航点并记录偏差
        """
        for idx, wp in enumerate(self.waypoints):
            # 步骤1: 导航到当前点位
            rospy.loginfo(f"导航到第{idx+1}个点: ({wp['x']}, {wp['y']})")
            nav_result = self.send_nav_goal(wp['x'], wp['y'], wp['yaw'])
            
            # 检查导航结果
            if nav_result != 3:  # 3表示导航成功
                rospy.logwarn("导航失败，跳过该点")
                self.calibration_results.append({
                    'target': wp,
                    'actual': None,
                    'deviation': None
                })
                continue
            
            # 步骤2: 获取实际到达位置
            actual_pos = self.get_current_position()
            if actual_pos is None:
                rospy.logwarn("无法获取实际位置，跳过该点")
                self.calibration_results.append({
                    'target': wp,
                    'actual': None,
                    'deviation': None
                })
                continue
            
            # 步骤3: 计算偏差
            deviation = {
                'x': actual_pos['x'] - wp['x'],
                'y': actual_pos['y'] - wp['y'],
                'yaw': actual_pos['yaw'] - wp['yaw']
            }
            
            # 记录结果
            self.calibration_results.append({
                'target': wp,
                'actual': actual_pos,
                'deviation': deviation
            })
            
            rospy.loginfo(f"目标位置: ({wp['x']}, {wp['y']})")
            rospy.loginfo(f"实际位置: ({actual_pos['x']}, {actual_pos['y']})")
            rospy.loginfo(f"偏差: ({deviation['x']}, {deviation['y']})")
        
        # 生成校准参数文件
        self.generate_calibration_file()
        
        rospy.loginfo("全部点位校准完成")
    
    def generate_calibration_file(self):
        """
        生成校准参数文件
        """
        filename = "calibration_results.json"
        
        # 准备写入的数据
        data = {
            'waypoints_count': len(self.waypoints),
            'results': self.calibration_results
        }
        
        # 写入JSON文件
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
        
        rospy.loginfo(f"校准结果已保存到 {filename}")


def main():
    """
    主程序入口
    """
    try:
        # 创建节点实例
        node = MultiPointNavigationCalibrator()
        # 启动主运行循环
        node.run()
    except rospy.ROSInterruptException:
        # ROS中断异常处理
        pass


if __name__ == '__main__':
    main()
