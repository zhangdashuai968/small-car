'''
阻尼模式下读取
1. 舵机角度
2. 关节角度
3. 末端位姿
'''
import time
import math
import numpy as np
# 阿凯机器人工具箱
from kyle_robot_toolbox.robot_arm.arm4dof_uservo \
	import Arm4DoFUServo

# 打印夹爪的角度
PRINT_GRIPPER_ANGLE = False
# 打印舵机原始角度
PRINT_SERVO_ANGLE = False
# 打印关节角度
PRINT_JOINT_ANGLE = True
# 打印末端姿态
PRINT_END_POSE = False

# 创建机械臂
arm = Arm4DoFUServo(config_folder="./config", is_init_pose=False)
# 设置为阻尼模式
# 数值越大，阻尼越大
arm.set_damping(1000)

# 不断查询舵机角度
# 不停的打印当前原始舵机角度
while True:
	if PRINT_SERVO_ANGLE:
		# 获取原始角度列表
		angle_list = arm.get_servo_angle_list()
		# 打印原始舵机角度
		a1, a2, a3, a4, a5, a6 = angle_list
		print("舵机原始角度")
		print(f"J1: {a1:.1f} J2: {a2:.1f} J3: {a3:.1f} J4: {a4:.1f} J5: {a5:.1f} J6: {a6:.1f}")
		if PRINT_GRIPPER_ANGLE:
			angle_list = angle_list[:-1]
		angle_list_str = ", ".join([f"{v:.1f}" for v in angle_list])
		print(f"舵机原始角度列表: [{angle_list_str}]")
	
	if PRINT_JOINT_ANGLE:
		# 打印机械臂关节角度
		joint_angle_list = arm.get_joint_angle_list()
		# 转换为角度
		joint_angle_list = np.degrees(np.float32(joint_angle_list))
		a1, a2, a3, a4, a5, a6 = joint_angle_list
		print("机械臂关节角度")
		print(f" J1: {a1:.5f}\n J2: {a2:.5f}\n J3: {a3:.5f}\n" + \
			f" J4: {a4:.5f}\n J5: {a5:.5f}\n J6: {a6:.5f}\n")
		if not PRINT_GRIPPER_ANGLE:
			joint_angle_list = joint_angle_list[:-1]
		joint_angle_list_str =  ", ".join([f"{v:.1f}" for v in joint_angle_list])
		print(f"关节角度列表: [{joint_angle_list_str}]")

	if PRINT_END_POSE:
		# 打印当前机械臂的位姿
		x, y, z, pitch = arm.get_tool_pose()
		print("机械臂末端位姿")
		print(f"- 位置： X={x} Y={y} Z={z}")
		print(f"- 姿态: 俯仰角 {pitch:.5f} 横滚角 {roll:.5f}")
		
	time.sleep(1)