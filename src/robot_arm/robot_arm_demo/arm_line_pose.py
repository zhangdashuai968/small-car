'''
控制机械臂运动到目标位姿
'''
import sys
import logging
from absl import app
from absl import flags

# 阿凯机器人工具箱
from kyle_robot_toolbox.robot_arm.arm4dof_uservo import Arm4DoFUServo

# 设置日志等级
logging.basicConfig(level=logging.INFO)

def main(argv):
	'''主程序'''
	pose_name = FLAGS.pose_name
	logging.info(f"位姿名称: {pose_name}")
	# 创建机械臂对象
	logging.info("机械臂初始化")
	arm = Arm4DoFUServo(config_folder="./config")
	logging.info(f"控制机械臂运动到: {pose_name}")
	arm.set_tool_pose(pose_name=pose_name)

FLAGS = flags.FLAGS
flags.DEFINE_string('pose_name', 'line_detect', '机械臂位姿名称')

# 运行主程序
app.run(main)
