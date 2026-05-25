#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
多點導航校準腳本

功能：
- 自動遍歷所有導航點
- 記錄實際到達位置與目標位置的偏差
- 生成校準後的參數文件

使用方法：
1. 確保ROS環境已正確設置
2. 啟動機器人和導航相關節點
3. 運行此腳本：python multi_point_navigation_grasp.py
4. 腳本將自動導航到各個預設點位並記錄偏差
5. 校準結果將保存在calibration_results.json文件中

輸出文件格式：
- waypoints_count: 導航點總數
- results: 每個導航點的校準結果
  - target: 目標位置
  - actual: 實際到達位置
  - deviation: 偏差值

適用環境：
- Ubuntu 18.04 + ROS Melodic
- 麥克納姆輪底盤機器人
- 配備激光雷達

作者：ROS開發團隊
創建日期：2024年
版本：1.0
"""
import rospy
import actionlib
import json
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


class MultiPointNavigationGrasp:
    """
    多點導航抓取主控類
    
    負責協調機器人的導航、視覺識別和機械臂抓取操作
    """
    def __init__(self):
        """
        初始化多點導航抓取節點
        
        功能：
        - 初始化ROS節點
        - 連接move_base動作服務器
        - 等待並連接機械臂和視覺識別服務
        - 設置導航點位和放置位置參數
        """
        rospy.init_node('multi_point_navigation_grasp')
        
        # move_base動作客戶端 - 用於導航控制
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        # TF監聽器 - 用於獲取機器人當前位置
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 校準結果存儲
        self.calibration_results = []
        
        # 8個導航點位配置（示例值，需根據實際地圖調整）
        # 每個點位包含：坐標、朝向、是否需要抓取
        self.waypoints = [
            # 起始點（不抓取）
            {'x': 1.0, 'y': 1.0, 'yaw': 0.0, 'grasp': False},
            # 抓取點1-6
            {'x': 2.0, 'y': 1.0, 'yaw': 0.0, 'grasp': True},
            {'x': 3.0, 'y': 1.0, 'yaw': 0.0, 'grasp': True},
            {'x': 4.0, 'y': 1.0, 'yaw': 0.0, 'grasp': True},
            {'x': 5.0, 'y': 1.0, 'yaw': 0.0, 'grasp': True},
            {'x': 6.0, 'y': 1.0, 'yaw': 0.0, 'grasp': True},
            {'x': 7.0, 'y': 1.0, 'yaw': 0.0, 'grasp': True},
            # 結束點（不抓取）
            {'x': 8.0, 'y': 1.0, 'yaw': 0.0, 'grasp': False},
        ]

    def send_nav_goal(self, x, y, yaw):
        """
        發送導航目標到move_base
        
        參數：
        x (float): 目標點X坐標
        y (float): 目標點Y坐標
        yaw (float): 目標朝向（弧度）
        
        返回：
        int: 導航結果狀態碼
            - 3: 導航成功
            - 其他: 導航失敗
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # 使用map坐標系
        goal.target_pose.header.stamp = rospy.Time.now()  # 當前時間戳
        
        # 設置目標位置
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0  # 地面高度
        
        # 簡化：只設置yaw，四元數轉換略
        # 注意：實際應用中需要將yaw轉換為四元數
        goal.target_pose.pose.orientation.w = 1.0  # 默認朝向
        
        # 發送目標並等待結果
        result = self.move_base_client.send_goal_and_wait(goal)
        return result

    def get_current_position(self):
        """
        獲取機器人當前位置
        
        返回：
        dict: 包含當前位置信息的字典，包括x, y, yaw
        """
        try:
            # 獲取機器人在map坐標系下的位置
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
            
            # 提取位置信息
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # 簡化：只提取yaw，四元數轉換略
            # 注意：實際應用中需要將四元數轉換為yaw
            yaw = 0.0  # 默認值
            
            return {'x': x, 'y': y, 'yaw': yaw}
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("無法獲取機器人當前位置")
            return None
    
    def generate_calibration_file(self):
        """
        生成校準參數文件
        """
        filename = "calibration_results.json"
        
        # 準備寫入的數據
        data = {
            'waypoints_count': len(self.waypoints),
            'results': self.calibration_results
        }
        
        # 寫入JSON文件
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
        
        rospy.loginfo("校準結果已保存到 {}".format(filename))
    
    def run(self):
        """
        主運行循環：遍歷所有導航點並記錄偏差
        """
        for idx, wp in enumerate(self.waypoints):
            # 步驟1: 導航到當前點位
            rospy.loginfo("導航到第{}個點: ({}, {})".format(idx+1, wp['x'], wp['y']))
            nav_result = self.send_nav_goal(wp['x'], wp['y'], wp['yaw'])
            
            # 檢查導航結果
            if nav_result != 3:  # 3表示導航成功
                rospy.logwarn("導航失敗，跳過該點")
                self.calibration_results.append({
                    'target': wp,
                    'actual': None,
                    'deviation': None
                })
                continue
            
            # 步驟2: 獲取實際到達位置
            actual_pos = self.get_current_position()
            if actual_pos is None:
                rospy.logwarn("無法獲取實際位置，跳過該點")
                self.calibration_results.append({
                    'target': wp,
                    'actual': None,
                    'deviation': None
                })
                continue
            
            # 步驟3: 計算偏差
            deviation = {
                'x': actual_pos['x'] - wp['x'],
                'y': actual_pos['y'] - wp['y'],
                'yaw': actual_pos['yaw'] - wp['yaw']
            }
            
            # 記錄結果
            self.calibration_results.append({
                'target': wp,
                'actual': actual_pos,
                'deviation': deviation
            })
            
            rospy.loginfo("目標位置: ({}, {})".format(wp['x'], wp['y']))
            rospy.loginfo("實際位置: ({}, {})".format(actual_pos['x'], actual_pos['y']))
            rospy.loginfo("偏差: ({}, {})".format(deviation['x'], deviation['y']))
        
        # 生成校準參數文件
        self.generate_calibration_file()
        
        rospy.loginfo("全部點位校準完成")

if __name__ == '__main__':
    """
    主程序入口
    
    創建多點導航抓取節點實例並啟動運行
    """
    try:
        # 創建節點實例
        node = MultiPointNavigationGrasp()
        # 啟動主運行循環
        node.run()
    except rospy.ROSInterruptException:
        # ROS中斷異常處理
        pass