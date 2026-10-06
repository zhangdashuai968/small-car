#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
物流搬运系统

该模块实现了基于ROS的物流搬运机器人控制系统，主要功能包括：
- 自动导航到指定位置
- 检测并抓取目标物体
- 将物体搬运到指定位置并放置
- 自动返回终点

依赖项：
- ROS Melodic (Ubuntu 18.04)
- move_base_msgs
- geometry_msgs
- tf
- vl_locate服务
- std_msgs
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from vl_locate.srv import GetObject
from std_msgs.msg import Bool, String
import time

class CargoHandling():
    """
    物流搬运控制类
    
    该类负责控制物流搬运机器人的整个搬运流程，包括导航、物体检测、抓取、放置等操作。
    通过ROS消息和服务与底层硬件和感知系统进行交互。
    
    属性:
        waypoints (list): 路径点列表，包含抓取点和放置点的坐标
        end_pos (list): 任务完成后的返回终点坐标
        nav_offset (float): 导航偏移量，默认为0.2米
        move_base (actionlib.SimpleActionClient): move_base动作客户端，用于导航
        vlm_detection_srv (rospy.ServiceProxy): 目标检测服务代理
        grab_finished (bool): 抓取完成状态标志
        grab_pub (rospy.Publisher): 抓取命令发布者
    """
    def __init__(self):
        try:
            # 正确缩进的代码（4空格）
            self.waypoints = rospy.get_param('~waypoints')
            self.end_pos = rospy.get_param('~end_pos')
            self.nav_offset = rospy.get_param('~nav_offset', 0.2)
            
            # 初始化导航客户端
            self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            self.move_base.wait_for_server()
            
            # 等待目标识别服务
            rospy.wait_for_service('vlm_detection')
            self.vlm_detection_srv = rospy.ServiceProxy('vlm_detection', GetObject)
            
            # 订阅抓取完成状态
            self.grab_finished = False
            rospy.Subscriber('/grab_finished', Bool, self.grab_finished_callback)
            
            # 发布抓取命令
            self.grab_pub = rospy.Publisher('/grab', Bool, queue_size=1)
            
            # 发布步骤更新信息
            self.step_pub = rospy.Publisher('/update_step', String, queue_size=10)
            
            rospy.loginfo("物流搬运系统初始化完成")
        except Exception as e:
            rospy.logerr("__init__方法中发生致命错误: %s", str(e))
            import traceback
            rospy.logerr(traceback.format_exc())  # 打印完整堆栈
            raise  # 或者直接 sys.exit(1)
    
    def grab_finished_callback(self, msg):
        """
        抓取完成状态回调函数
        
        该方法作为ROS订阅者回调函数，用于接收抓取完成状态消息，并更新
        本地的抓取完成状态标志。
        
        参数:
            msg (std_msgs.Bool): 抓取完成状态消息，True表示抓取完成，False表示未完成
        
        返回:
            无
        """
        self.grab_finished = msg.data
    
    def send_goal(self, x, y, yaw):
        """
        发送导航目标
        
        该方法创建一个导航目标并发送给move_base动作服务器，实现机器人到指定位置的导航。
        
        参数:
            x (float): 目标位置的x坐标
            y (float): 目标位置的y坐标
            yaw (float): 目标位置的偏航角（弧度）
        
        返回:
            bool: 导航是否成功，True表示成功，False表示失败
        
        异常:
            无
        """
        try:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = Point(x, y, 0)
            quat = quaternion_from_euler(0, 0, yaw)
            goal.target_pose.pose.orientation = Quaternion(*quat)
            
            self.move_base.send_goal(goal)
            finished = self.move_base.wait_for_result(rospy.Duration(60))
            
            if not finished:
                rospy.logerr("导航超时!")
                return False
            else:
                state = self.move_base.get_state()
                if state == 3:  # SUCCEEDED
                    rospy.loginfo("导航成功!")
                    return True
                else:
                    rospy.logerr("导航失败!")
                    return False
        except Exception as e:
            rospy.logerr("send_goal方法中发生致命错误: %s", str(e))
            import traceback
            rospy.logerr(traceback.format_exc())  # 打印完整堆栈
            raise  # 或者直接 sys.exit(1)
            return False
    
    def detect_and_grab(self, object_name):
        """
        检测并抓取物体
        
        该方法调用目标识别服务检测指定物体并发送抓取命令，不检测抓取是否成功。
        
        参数:
            object_name (str): 要检测和抓取的物体名称
        
        返回:
            bool: 始终返回True
        
        异常:
            rospy.ServiceException: 当调用目标识别服务失败时抛出
        """
        try:
            # 调用目标识别服务前设置抓取模式
            rospy.set_param('/grab', 1)
            rospy.loginfo("切换抓取模式: /grab = 1")
            rospy.sleep(0.2)
            
            # 调用目标识别服务
            response = self.vlm_detection_srv(object_name)
            if response.success:
                rospy.loginfo("目标识别成功: %s", response.message)
            else:
                rospy.logwarn("目标识别失败: %s", response.message)
            
            # 发送抓取命令
            self.grab_pub.publish(True)
            rospy.loginfo("已发送抓取命令，等待5秒...")
            
            # 等待5秒
            rospy.sleep(5.0)
            rospy.loginfo("等待完成，继续执行")
            
            return True
        except rospy.ServiceException as e:
            rospy.logerr("调用目标识别服务失败: %s", str(e))
            return True
        except Exception as e:
            rospy.logerr("detect_and_grab方法中发生致命错误: %s", str(e))
            import traceback
            rospy.logerr(traceback.format_exc())  # 打印完整堆栈
            raise  # 或者直接 sys.exit(1)
            return True
    
    def place_object(self, object_name):
        """
        放置物体
        
        该方法通过设置/grab参数为0，然后调用vlm_detection服务来执行放置操作。
        在抓取动作2点执行切换放置模式（/grab=0）的放置动作。
        
        参数:
            object_name (str): 物体名称
        
        返回:
            bool: 始终返回True
        
        异常:
            无
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
        
        异常:
            无
        """
        try:
            # 发布初始步骤信息
            step_msg = String()
            step_msg.data = "任务开始"
            self.step_pub.publish(step_msg)
            
            # 从参数服务器获取物体名称列表，如果未设置则使用默认值
            object_names = rospy.get_param('~object_names', [u"蓝方块", u"红方块", u"绿方块"])
            
            # 遍历所有点位
            for i in range(0, len(self.waypoints), 2):
                # 发布步骤信息
                step_msg.data = u"前往抓取点"
                self.step_pub.publish(step_msg)
                
                # 导航到抓取点
                grab_waypoint = self.waypoints[i]
                rospy.loginfo(u"前往抓取点: %s", str(grab_waypoint))
                if not self.send_goal(grab_waypoint[0], grab_waypoint[1], grab_waypoint[2]):
                    rospy.logerr(u"无法到达抓取点，跳过此任务")
                    continue
                
                # 发布步骤信息
                step_msg.data = u"检测并抓取物体"
                self.step_pub.publish(step_msg)
                
                # 检测并抓取物体（不管成功与否都继续）
                object_name = object_names[i//2] if i//2 < len(object_names) else "物体"
                self.detect_and_grab(object_name)
                
                # 发布步骤信息
                step_msg.data = "前往放置点(抓取动作2点)"
                self.step_pub.publish(step_msg)
                
                # 导航到抓取动作2点作为放置点
                place_waypoint = self.waypoints[i+1]
                rospy.loginfo(u"前往放置点(抓取动作2点): %s", str(place_waypoint))
                if not self.send_goal(place_waypoint[0], place_waypoint[1], place_waypoint[2]):
                    rospy.logerr(u"无法到达放置点(抓取动作2点)")
                    continue
            
            # 发布步骤信息
            step_msg.data = u"放置物体"
            self.step_pub.publish(step_msg)
            
            # 放置物体
            self.place_object(object_name)
        
            # 发布步骤信息
            step_msg.data = "返回终点"
            self.step_pub.publish(step_msg)
            
            # 返回终点
            rospy.loginfo("返回终点: %s", str(self.end_pos))
            if self.send_goal(self.end_pos[0], self.end_pos[1], self.end_pos[2]):
                # 发布步骤信息
                step_msg.data = u"任务完成"
                self.step_pub.publish(step_msg)
                rospy.loginfo(u"任务完成!")
            else:
                # 发布步骤信息
                step_msg.data = u"无法返回终点"
                self.step_pub.publish(step_msg)
                rospy.logerr(u"无法返回终点")
                
        except Exception as e:
            rospy.logerr("run方法中发生致命错误: %s", str(e))
            import traceback
            rospy.logerr(traceback.format_exc())  # 打印完整堆栈
            raise  # 或者直接 sys.exit(1)

def main():
    """
    ROS节点主函数
    
    该函数初始化ROS节点，创建CargoHandling实例，并启动物流搬运任务。
    这是程序的入口点，负责协调整个物流搬运流程。
    
    参数:
        无
    
    返回:
        无
    
    异常:
        rospy.ROSException: 当ROS节点初始化失败时抛出
    """
    try:
        rospy.init_node('cargo_handling')
        handler = CargoHandling()
        handler.run()
    except Exception as e:
        rospy.logerr("cargo_handling.py 发生致命错误: %s", str(e))
        import traceback
        rospy.logerr(traceback.format_exc())  # 打印完整堆栈
        raise  # 或者直接 sys.exit(1)


if __name__ == '__main__':
    main()