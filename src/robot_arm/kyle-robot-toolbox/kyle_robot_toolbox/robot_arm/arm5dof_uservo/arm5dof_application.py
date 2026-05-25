'''
五自由度机械臂应用
----------------------------------------------
@作者: 阿凯爱玩机器人
@微信: xingshunkai
@邮箱: xingshunkai@qq.com
@B站: https://space.bilibili.com/40344504
'''
import os
import math
import numpy as np
import yaml
import time

# 阿凯机器人工具箱
from kyle_robot_toolbox.transform import Transform

class Arm5DoFApplication:
	'''五自由度机械臂应用'''
	GRIPPER_OPEN_ANGLE = np.radians(30)     # 夹爪张开角度
	GRAB_CUBE_DZ = 5.0                    	# 抓取点在物体坐标系基础上Z轴偏移量
	LIFT_UP_DZ = 50                        	# 移动到上方Z轴平移距离
	def __init__(self, arm, T_arm2ws=None, config_folder="config", is_debug=False):
		self.is_debug = is_debug
		self.arm = arm
		if T_arm2ws is None:
			file_path = os.path.join(config_folder, "handeye_calibration", "T_arm2ws.txt")
			self.T_arm2ws = np.loadtxt(file_path, delimiter=",")
		else:
			self.T_arm2ws = T_arm2ws
		self.T_ws2arm = Transform.inverse(self.T_arm2ws)
	   	# 载入工件平面的高度
		object_config = None
		yaml_path = os.path.join(config_folder, "object.yaml")
		with open(yaml_path, 'r', encoding='utf-8') as f:
			object_config = yaml.load(f.read(), Loader=yaml.SafeLoader)
		self.object_height = object_config["height"]
		# 设置一个默认的工作台Z轴高度
		self.wz_default = self.object_height
	
	def tf_ws2arm(self, wx, wy, wz=None):
		'''空间变换, 将坐标由工作台坐标系转换为机械臂坐标系'''
		if wz is None:
			# 将wz设置为物体高度的1/2
			wz = self.object_height * 0.5
		w_vect = np.float32([wx, wy, wz, 1])
		arm_vect = self.T_arm2ws.dot(w_vect)
		arm_x, arm_y, arm_z = arm_vect.reshape(-1)[:3]
		return (arm_x, arm_y, arm_z)

	def tf_arm2ws(self, ax, ay, az):
		'''空间变换, 将坐标由机械臂坐标系转换为工作台坐标系'''
		arm_vect = np.float32([ax, ay, az, 1])
		ws_vect = self.T_ws2arm.dot(arm_vect)
		wx, wy, wz = ws_vect.reshape(-1)[:3]
		return [wx, wy, wz]

	def move2release_pose(self, pose_name="object_release"):
		'''运动到释放位姿'''
		# 获取物体释放的位姿
		theta = np.radians(self.arm.pose_dict[pose_name])
		# 正向运动学得到目标位姿
		[x6, y6, z6, pitch, roll] = self.arm.kinematic.forward_kinematic_v2(theta)
		# 先运动到目标位姿上方
		self.arm.set_tool_pose([x6, y6, z6+self.LIFT_UP_DZ], T=1.0)
		# 运动到目标位置
		self.arm.set_tool_pose([x6, y6, z6], T=0.4)

	def grab_cubic(self, wx, wy, wz):
		'''抓取平台上的一个物块'''
		# 添加偏移量
		wz += self.GRAB_CUBE_DZ
		self.gripper_open()
		arm_x, arm_y, arm_z  = self.tf_ws2arm(wx, wy, wz)
		theta0 = math.atan2(arm_y, arm_x)
		self.arm.set_joint_angle_soft("joint1", theta0, T=1.0)
		self.move2ws(wx, wy, wz+self.LIFT_UP_DZ, t=1.0)
		self.move2ws(wx, wy, wz, t=0.4)
		self.gripper_close()
		self.move2ws(wx, wy, wz+self.LIFT_UP_DZ, t=1.0)
		# 移动到物块释放区
		self.move2release_pose(pose_name="object_release")
		self.gripper_open()
		# 先运行到Home姿态，可以提高精度
		self.home()
		# 移动到画面拍摄位姿
		self.arm.set_tool_pose(pose_name="capture_image")
		
	def move2ws(self, wx, wy, wz=None, t=1.0):
		if wz is None:
			# 将wz设置为物体高度的1/2
			wz =  self.wz_default
		
		(arm_x, arm_y, arm_z) = self.tf_ws2arm(wx, wy, wz)
		if self.is_debug:
			print(f"运动到 wx={wx:.2f}, wy={wy:.2f}, wz={wz:.2f}")
			print(f"运动到 arm_x={arm_x:.2f}, arm_y={arm_y:.2f}, wz={arm_z:.2f}")
		# 设置机械臂末端位姿
		self.arm.set_tool_pose([arm_x, arm_y, arm_z], T=t)
	
	def move2ws_top(self, wx, wy, wz=None, dz=None, t=0.20):
		if wz is None:
			# 将wz设置为物体高度的1/2
			wz =  self.wz_default
		if dz is None:
			dz = self.MOVE_TOP_DZ
		arm_x, arm_y, arm_z  = self.tf_ws2arm(wx, wy, wz+dz)
		self.arm.set_tool_pose([arm_x, arm_y, arm_z], T=t)
	
	def lift_up(self, dz=None, t=0.80):
		'''抬起物块'''
		if dz is None:
			dz = self.MOVE_TOP_DZ
		x, y, z, pitch, roll = self.arm.get_tool_pose()
		print("抬起物块")
		self.arm.set_tool_pose([x, y, z+dz], T=t)
	
	def gripper_open(self, angle=None):
		'''爪子打开'''
		if angle is None:
			angle = self.GRIPPER_OPEN_ANGLE
		self.arm.gripper_open(angle=angle, T=0.5)
	
	def gripper_close(self):
		'''爪子闭合'''
		self.arm.gripper_close(T=0.5)
		
	def home(self):
		'''初始位姿'''
		self.arm.home()
		
	
