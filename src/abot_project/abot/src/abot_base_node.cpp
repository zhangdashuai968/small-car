#include <ros/ros.h>         // 包含ROS核心库头文件
#include "abot_base.h"       // 包含机器人基础功能头文件

/**
 * 主函数 - 机器人基础节点的入口点
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 */
int main(int argc, char** argv )
{
    // 初始化ROS节点，设置节点名称为"riki_base_node"
    ros::init(argc, argv, "riki_base_node");
    
    // 创建RikiBase类的实例，用于控制机器人的基础功能
    RikiBase riki;
    
    // 进入ROS事件循环，等待并处理订阅的消息
    // 此函数会持续运行直到节点被关闭
    ros::spin();
    
    // 返回0表示程序正常退出
    return 0;
}
