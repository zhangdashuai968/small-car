#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
物流搬运系统主程序

该程序负责控制机器人完成完整的物流搬运任务，包括：
1. 导航到指定的抓取点
2. 检测并抓取指定物体
3. 导航到指定的放置点
4. 放置物体
5. 最后返回终点位置

依赖项：
- ROS (Robot Operating System)
- move_base (用于路径规划和导航)
- vl_locate (用于目标检测)
- 相关的服务和消息类型定义
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf
from vl_locate.srv import DetectObject, DetectObjectRequest
from std_msgs.msg import String
import time
import traceback

class CargoHandling:
    """
    物流搬运类
    
    该类封装了物流搬运任务的所有功能，包括导航、物体检测、抓取和放置。
    
    属性:
        client (actionlib.SimpleActionClient): 用于与move_base动作服务器通信
        vlm_detection_srv (rospy.ServiceProxy): 用于调用目标检测服务
        waypoints (list): 包含所有路径点的列表
        object_names (list): 包含所有要抓取物体名称的列表
        end_pos (list): 终点位置
        step_pub (rospy.Publisher): 用于发布当前步骤信息
    """
    
    def __init__(self):
        """
        初始化物流搬运系统
        """
        try:
            # 获取参数
            self.waypoints = rospy.get_param('~waypoints', [])
            self.object_names = rospy.get_param('~object_names', [])
            self.end_pos = rospy.get_param('~end_pos', [0, 0, 0])
            
            # 创建move_base客户端
            self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            rospy.loginfo(u"等待move_base动作服务器...")
            self.client.wait_for_server()
            rospy.loginfo(u"已连接到move_base动作服务器")
            
            # 创建目标检测服务代理
            rospy.wait_for_service('vlm_detection')
            self.vlm_detection_srv = rospy.ServiceProxy('vlm_detection', DetectObject)
            
            # 创建步骤信息发布者
            self.step_pub = rospy.Publisher('/update_step', String, queue_size=10)
            
        except Exception as e:
            rospy.logerr(u"初始化过程中发生错误: %s", str(e))
            import traceback
            rospy.logerr(traceback.format_exc())
            raise
    
    def grab_finished_callback(self, data):
        """
        抓取完成回调函数
        
        当抓取完成后，该函数会被调用。
        
        参数:
            data (String): 包含抓取结果的消息
        """
        rospy.loginfo(u"收到抓取完成信号: %s", data.data)
    
    def send_goal(self, x, y, yaw):
        """
        发送导航目标
        
        该方法向move_base发送一个导航目标，并等待结果。
        
        参数:
            x (float): 目标点的x坐标
            y (float): 目标点的y坐标
            yaw (float): 目标点的朝向（弧度）
        
        返回:
            bool: 如果成功到达目标点则返回True，否则返回False
        """
        try:
            # 创建目标
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = Point(x, y, 0)
            
            # 将欧拉角转换为四元数
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            goal.target_pose.pose.orientation = Quaternion(*quat)
            
            # 发送目标并等待结果
            self.client.send_goal(goal)
            self.client.wait_for_result()
            
            # 检查结果
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(u"成功到达目标点: (%.2f, %.2f, %.2f)", x, y, yaw)
                return True
            else:
                rospy.logerr(u"未能到达目标点: (%.2f, %.2f, %.2f)", x, y, yaw)
                return False
        except Exception as e:
            rospy.logerr(u"send_goal方法中发生错误: %s", str(e))
            import traceback
            rospy.logerr(traceback.format_exc())
            return False
    
    def detect_and_grab(self, object_name):
        """
        检测并抓取物体
        
        该方法首先调用目标检测服务找到物体，然后发送抓取命令。
        
        参数:
            object_name (str): 要抓取的物体名称
        
        返回:
            bool: 如果成功抓取物体则返回True，否则返回False
        """
        try:
            rospy.loginfo(u"开始检测并抓取物体: %s", object_name)
            
            # 设置抓取模式为1
            rospy.set_param('/grab', 1)
            rospy.loginfo(u"切换抓取模式: /grab = 1")
            rospy.sleep(0.2)
            
            # 调用目标检测服务
            request = DetectObjectRequest()
            request.object_name = object_name
            response = self.vlm_detection_srv(request)
            
            if response.success:
                rospy.loginfo(u"检测到物体: %s", response.message)
                # 发布抓取命令
                rospy.loginfo(u"发送抓取命令")
                return True
            else:
                rospy.logwarn(u"未检测到物体: %s", response.message)
                return False
        except rospy.ServiceException as e:
            rospy.logerr(u"调用目标检测服务失败: %s", str(e))
            return False
        except Exception as e:
            rospy.logerr(u"detect_and_grab方法中发生致命错误: %s", str(e))
            import traceback
            rospy.logerr(traceback.format_exc())
            return False
    
    def place_object(self, object_name):
        """
        放置物体
        
        该方法在指定位置放置物体。
        
        参数:
            object_name (str): 要放置的物体名称
        
        返回:
            bool: 如果成功放置物体则返回True，否则返回False
        """
        try:
            rospy.loginfo(u"在抓取动作2点放置物体: %s", object_name.encode('utf-8') if isinstance(object_name, unicode) else object_name)
            # 设置抓取模式为0
            rospy.set_param('/grab', 0)
            rospy.loginfo(u"切换放置模式: /grab = 0")
            rospy.sleep(0.2)
            
            # 调用目标识别服务，使用固定物体名称'图片中心点'
            response = self.vlm_detection_srv('图片中心点')
            if response.success:
                rospy.loginfo(u"放置服务调用成功: %s", response.message.encode('utf-8') if isinstance(response.message, unicode) else response.message)
            else:
                rospy.logwarn("放置服务调用失败: %s", response.message)
        except rospy.ServiceException as e:
            rospy.logerr("调用放置服务失败: %s", str(e))
        except Exception as e:
            rospy.logerr("place_object方法中发生致命错误: %s", str(e))
            import traceback
            rospy.logerr(traceback.format_exc())  # 打印完整堆栈
            raise  # 或者直接 sys.exit(1)
        
        rospy.loginfo("等待5秒...")
        rospy.sleep(5.0)
        rospy.loginfo("物体放置完成!")
        return True
    
    def run(self):
        """
        主循环
        
        该方法执行完整的物流搬运任务，包括遍历所有路径点，依次导航到抓取点、
        检测并抓取物体、导航到放置点、放置物体，最后返回终点。
        
        参数:
            无
        
        返回:
            无
        """
        try:
            # 发布初始步骤信息
            step_msg = String()
            step_msg.data = u"任务开始"
            self.step_pub.publish(step_msg)
            
            # 获取所有物体名称
            object_names = self.object_names
            
            # 遍历所有路径点
            for i in range(0, len(self.waypoints), 2):
                # 检查是否有足够的路径点
                if i + 1 >= len(self.waypoints):
                    rospy.logwarn(u"路径点不足，跳过最后一个不完整的任务")
                    break
                
                # 获取抓取点和放置点
                grab_waypoint = self.waypoints[i]
                place_waypoint = self.waypoints[i + 1]
                
                # 确保有对应的物体名称
                if i // 2 >= len(object_names):
                    rospy.logwarn(u"物体名称不足，跳过任务")
                    break
                
                object_name = object_names[i // 2]
                
                # 发布当前步骤信息
                step_msg.data = u"前往抓取点: %s" % object_name
                self.step_pub.publish(step_msg)
                
                # 导航到抓取点
                if not self.send_goal(grab_waypoint[0], grab_waypoint[1], grab_waypoint[2]):
                    rospy.logerr(u"无法到达抓取点，跳过此任务")
                    continue
                
                # 检测并抓取物体
                if not self.detect_and_grab(object_name):
                    rospy.logerr(u"无法抓取物体，跳过此任务")
                    continue
                
                # 发布当前步骤信息
                step_msg.data = u"前往放置点(抓取动作2点)"
                self.step_pub.publish(step_msg)
                
                # 导航到放置点
                if not self.send_goal(place_waypoint[0], place_waypoint[1], place_waypoint[2]):
                    rospy.logerr(u"无法到达放置点，跳过此任务")
                    continue
                
                # 放置物体
                self.place_object(object_name)
            
            # 发布最终步骤信息
            step_msg.data = u"返回终点"
            self.step_pub.publish(step_msg)
            
            # 返回终点
            self.send_goal(self.end_pos[0], self.end_pos[1], self.end_pos[2])
            
            # 发布任务完成信息
            step_msg.data = u"任务完成"
            self.step_pub.publish(step_msg)
            
        except Exception as e:
            rospy.logerr(u"运行过程中发生致命错误: %s", str(e))
            import traceback
            rospy.logerr(traceback.format_exc())
            # 发布任务失败信息
            step_msg = String()
            step_msg.data = u"任务失败"
            self.step_pub.publish(step_msg)
            raise
    
    def main():
        """
        主函数
        
        初始化ROS节点，创建CargoHandling实例并运行主循环。
        """
        try:
            rospy.init_node('cargo_handling')
            handler = CargoHandling()
            handler.run()
        except Exception as e:
            rospy.logerr(u"主函数中发生致命错误: %s", str(e))
            import traceback
            rospy.logerr(traceback.format_exc())
            raise
    
    if __name__ == '__main__':
        main()