#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import roslaunch
import subprocess
import time
import os
import signal
import sys

class AutoMapSave:
    """自动建图并保存的ROS节点
    
    该脚本启动SLAM建图，然后在指定时间后自动保存地图
    """
    def __init__(self):
        rospy.init_node('auto_map_save', anonymous=True)
        
        # 设置参数
        self.map_save_time = rospy.get_param('~map_save_time', 120)  # 默认建图120秒后保存
        self.map_name = rospy.get_param('~map_name', 'auto_map')
        self.map_directory = rospy.get_param('~map_directory', '/home/abot/catkin_ws/src/abot_project/abot/maps')
        
        # 启动SLAM节点
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        # 尝试多个可能的路径来找到lidar_slam.launch文件
        possible_paths = [
            "/home/abot/catkin_ws/src/abot_project/abot/launch/slam/lidar_slam.launch",
            os.path.join(os.path.expanduser("~"), "catkin_ws/src/abot_project/abot/launch/slam/lidar_slam.launch"),
            os.path.join(os.path.dirname(__file__), "../launch/slam/lidar_slam.launch"),
            os.path.join(os.path.dirname(__file__), "../../abot/launch/slam/lidar_slam.launch")
        ]
        
        slam_launch_file = None
        for path in possible_paths:
            if os.path.exists(path):
                slam_launch_file = path
                break
        
        if slam_launch_file is None:
            rospy.logerr("Could not find lidar_slam.launch file in any of the expected locations.")
            raise FileNotFoundError("lidar_slam.launch file not found")
        
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [slam_launch_file])
        
    def start_slam(self):
        """启动SLAM建图"""
        rospy.loginfo("Starting SLAM mapping...")
        self.launch.start()
        rospy.loginfo("SLAM mapping started successfully.")
        
    def stop_slam(self):
        """停止SLAM建图"""
        rospy.loginfo("Stopping SLAM mapping...")
        self.launch.shutdown()
        rospy.loginfo("SLAM mapping stopped.")
        
    def save_map(self):
        """保存地图"""
        rospy.loginfo("Saving map...")
        
        # 确保地图保存目录存在
        if not os.path.exists(self.map_directory):
            os.makedirs(self.map_directory)
        
        # 切换到地图保存目录
        original_dir = os.getcwd()
        os.chdir(self.map_directory)
        
        try:
            # 使用map_server的map_saver保存地图
            result = subprocess.run(['rosrun', 'map_server', 'map_saver', '-f', self.map_name], 
                                  check=True, capture_output=True, text=True)
            rospy.loginfo("Map saved successfully as {}".format(self.map_name))
            rospy.loginfo("Output: {}".format(result.stdout))
        except subprocess.CalledProcessError as e:
            rospy.logerr("Failed to save map: {}".format(e))
            rospy.logerr("Error output: {}".format(e.stderr))
        finally:
            # 恢复原始目录
            os.chdir(original_dir)
        
    def run(self):
        """运行自动建图和保存流程"""
        rospy.loginfo("Starting automatic mapping and saving process...")
        
        try:
            # 启动SLAM建图
            self.start_slam()
            
            # 等待指定时间进行建图
            rospy.loginfo("Mapping for {} seconds...".format(self.map_save_time))
            time.sleep(self.map_save_time)
            
            # 停止SLAM建图
            self.stop_slam()
            
            # 保存地图
            self.save_map()
            
            rospy.loginfo("Automatic mapping and saving process completed.")
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Process interrupted by user.")
            self.stop_slam()
        except Exception as e:
            rospy.logerr("An error occurred: {}".format(e))
            self.stop_slam()


def main():
    """主函数"""
    try:
        auto_mapper = AutoMapSave()
        auto_mapper.run()
    except rospy.ROSInitException:
        rospy.logerr("Failed to initialize ROS node.")
    except Exception as e:
        rospy.logerr("An error occurred in main: {}".format(e))


if __name__ == '__main__':
    main()