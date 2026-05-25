'''
串口总线舵机五自由度运动学求解器
----------------------------------------------
@作者: 阿凯爱玩机器人
@微信: xingshunkai
@邮箱: xingshunkai@qq.com
@B站: https://space.bilibili.com/40344504
'''
import math
import numpy as np
# 阿凯机器人工具箱
from kyle_robot_toolbox.transform import *

# 设置Numpy打印精度
np.set_printoptions(precision=3, suppress=True)

class Arm5DoFKinematic:
	'''Arm5DoF协作机械臂运动学'''

	# 关节角度下限, 单位: 弧度
	JOINT_ANGLE_LOWERB = np.float64([-1.5708, -2.65, -1.5708, -2.0944, -1.5708])
	# 关节角度上限制, 单位: 弧度
	JOINT_ANGLE_UPPERB = np.float64([1.5708, 1.5708, 2.18166, 2.0944, 1.5708])
	# 默认关节角度 单位: 弧度
	# 注: 角度转弧度 
	JOINT_ANGLE_DEFAULT = np.radians([0, -90.0, 90.0, 0, 0.0, 0.0])
	# 关节编号
	JOINT1 = 0
	JOINT2 = 1
	JOINT3 = 2
	JOINT4 = 3
	JOINT6 = 4
	# DH表 连杆长度定义, 单位mm
	# 注: 参数仅供参考, 根据自己机械臂的连杆参数进行调整
	D1 = 80 
	A1 = 0.00
	A2 = 73.15
	A3 = 73.15
	A4 = 210.00
	# 预定义常量
	PI = math.pi
	MINUS_PI = - math.pi
	PI_M_2 = math.pi * 2.0 # pi*2
	PI_D_2 = math.pi / 2.0 # pi/2
	MIMUS_PI_D_2 = -1.0 * PI_D_2 # -pi/2
	
	def __init__(self, is_debug=False):
		'''初始化'''
		self.is_debug = is_debug
		# 当前关节角度赋值为默认角度
		self.cur_joint_angle = np.copy(self.JOINT_ANGLE_DEFAULT)
        
	def forward_kinematic_v1(self, joint_angle):
		'''正向运动学-工具坐标系'''
		# 提取关节角度
		theta1, theta2, theta3, theta4, theta6 = joint_angle
		# 关节与关节之间的变换
		T01 = Transform.dhmat(0, 0, theta1, self.D1)
		T12 = Transform.dhmat(self.MIMUS_PI_D_2, self.A1, theta2, 0)
		T23 = Transform.dhmat(0, self.A2, theta3, 0)
		T34 = Transform.dhmat(0, self.A3, theta4, 0)
		T45 = Transform.dhmat(0, self.A4, self.MIMUS_PI_D_2, 0)
		T56 = Transform.dhmat(self.MIMUS_PI_D_2, 0, theta6, 0)
		# 计算机械臂基坐标系到腕关节坐标系的变换
		T02 = T01.dot(T12)
		T03 = T02.dot(T23)
		T04 = T03.dot(T34)
		T05 = T04.dot(T45)
		T06 = T05.dot(T56)
		# 创建位姿对象
		pose = Pose()
		pose.set_transform_matrix(T06)
		return pose, [T01, T02, T03, T04, T05, T06]

	def forward_kinematic_v2(self, joint_angle, return_type="XYZ_Pitch_Roll"):
		'''正向运动学-工具坐标系
  		@joint_angle: 关节角度
		@return_type: 返回类型
			- "Pose": 返回Pose对象
			- "XYZ_Pitch_Roll": 返回[x6, y6, z6, pitch, roll] 
    	'''
		# 提取关节角度
		theta1, theta2, theta3, theta4, theta6 = joint_angle
		# 复合角度
		theta23 = theta2 + theta3
		theta234 = theta23 + theta4
		# 预先计算三角函数
		s1, s2, s3, s4, s6 = np.sin(joint_angle)
		c1, c2, c3, c4, c6 = np.cos(joint_angle)
		s23, c23 = np.sin(theta23), np.cos(theta23)
		s234, c234 = np.sin(theta234), np.cos(theta234)
		# 三维坐标
		x60 = self.A1 + self.A2*c2 + self.A3*c23 + self.A4*c234
		x6 = x60 * c1
		y6 = x60 * s1
		z6 = self.D1 - self.A2*s2 - self.A3*s23 - self.A4*s234
		if return_type == "Pose":
			# 求解旋转矩阵
			r11 = s1*s6 + c1*s234*c6
			r12 = s1*c6 - c1*s234*s6
			r13 = c1*c234
			r21 = s1*s234*c6 - c1*s6
			r22 = - s1*s234*s6 - c1*c6
			r23 = s1*c234
			r31 = c234*c6
			r32 = - c234*s6
			r33 = - s234
			# 创建位姿对象
			pose = Pose()
			# 构造旋转矩阵
			rmat = np.float32([
				[r11, r12, r13],
				[r21, r22, r23],
				[r31, r32, r33]])
			pose.set_rotation_matrix(rmat)
			pose.set_position(x6, y6, z6)
			return pose
		else:
			# 返回[x6, y6, z6, pitch]
			# 计算俯仰角Pitch
			# beta = \theta_{234} + \pi/2
			pitch = theta234 + self.PI_D_2
			if pitch > self.PI_M_2:
				pitch -= self.PI_M_2
			# 计算横滚角Roll
			# gamma = \theta_6 + \pi
			roll = theta6 + self.PI
			if roll > self.PI:
				roll -= self.PI_M_2
			return [x6, y6, z6, pitch, roll]
	
	def angle_unique(self, angle_list):
		'''得到不重复的角度列表'''
		return np.unique(np.round(np.float32(angle_list), 8))
	
	def is_joint_angle_legal(self, theta, joint_index, is_debug=None):
		'''判断关节角度是否合法'''
		if is_debug is None:
			is_debug = self.is_debug
		lowerb = self.JOINT_ANGLE_LOWERB[joint_index]
		upperb = self.JOINT_ANGLE_UPPERB[joint_index]
		is_legal = theta >= lowerb and theta <= upperb
		if is_debug:
			print(f"关节索引： {joint_index}")
			print(f"关节角度: {theta} 是否合法: {is_legal}")
			print(f"下限: {lowerb}, 上限: {upperb}")
		return is_legal
	
	def joint_angle_filter(self, theta_list, joint_index, is_debug=None):
		'''关节角度过滤器'''
		if is_debug is None:
			is_debug = self.is_debug
		# 过滤唯一的候选角度
		theta_list = np.unique(np.float64(theta_list))
		# 筛选后的候选关节角度
		theta_list_filter = []
		for theta in theta_list:
			if self.is_joint_angle_legal(theta, joint_index, is_debug):
				theta_list_filter.append(theta)
		if is_debug:
			print(f"关节索引： {joint_index}")
			print(f"过滤前的关节角度: {theta_list}")
			print(f"过滤后的关节角度:  {theta_list_filter}")
		return theta_list_filter
	
	def inverse_kinematic(self, x6, y6, z6, pitch, roll,\
     		last_joint_angle = None, is_debug=None):
		'''逆向运动学'''
		if is_debug is None:
			is_debug = self.is_debug
		# 上一次的关节角度
		if last_joint_angle is None:
			last_joint_angle = self.JOINT_ANGLE_DEFAULT
		# 候选关节角度
		candi_joint_angle_list = []
		# 求解theta6
		theta6 = roll - self.PI
		if theta6 < self.MINUS_PI:
			theta6 += self.PI_M_2
		if not self.is_joint_angle_legal(theta6, self.JOINT6, is_debug):
			return []
		# 求解theta234
		theta234 = pitch - self.PI_D_2
		c234 = math.cos(theta234)
		s234 = math.sin(theta234)
		# 求解关节1的角度
		theta1_list = None
		if x6 == 0.0 and y6 == 0.0:
			theta1_1 = last_joint_angle[self.JOINT1]
			theta1_list = [theta1_1]
		else:
			theta1_1 = math.atan2(y6, x6)
			theta1_2 = theta1_1 + np.pi
			if theta1_2 > np.pi:
				theta1_2 -= self.PI_M_2
			theta1_list = [theta1_1, theta1_2]
		for theta1 in self.joint_angle_filter(\
      		theta1_list, self.JOINT1, is_debug):
			c1 = math.cos(theta1)
			s1 = math.sin(theta1)
			# 求解关节6在关节1坐标系下的坐标
			x_16 = c1*x6 + s1*y6
			z_16 = z6 - self.D1
			# 定义b1, b2 
			b1 = x_16 - self.A1 - self.A4 * c234
			b2 = -z_16  - self.A4 * s234
			c3 = (b1**2 + b2**2 - self.A2**2 - self.A3**2) / (2*self.A2*self.A3)
			# 数值原因 cos(theta3)可能会再1.0附近
			# 为了被判定为非法, 因此做下特殊处理
			c3 = np.round(c3, 10)
			if is_debug:
				print(f"cos(theta3) = {c3}")
			s3_pw2 = np.round(1.0 - c3**2, 10)
			if np.abs(c3) > 1.0 or np.abs(s3_pw2) > 1.0:
				if is_debug:
					# 非法cos值, 工作区达不到
					print("非法cos值 工作区达不到")
				continue
			
			s3_abs = math.sqrt(s3_pw2)
			theta3_1 = math.atan2(s3_abs, c3)
			theta3_2 = math.atan2(-s3_abs, c3)
			# 求解theta2
			for theta3 in self.joint_angle_filter(\
				[theta3_1, theta3_2], self.JOINT3, is_debug):
				if is_debug:
					print(f"theta3 = {theta3}")
				K1 = self.A2 + self.A3 * c3
				K2 = self.A3 * math.sin(theta3)
				theta2 = math.atan2(b2, b1) - math.atan2(K2, K1)
				if not self.is_joint_angle_legal(theta2, self.JOINT2, is_debug):
					continue
				# 求解theta4
				theta4 = theta234 - theta2 - theta3
				if not self.is_joint_angle_legal(theta4, self.JOINT4, is_debug):
					continue
				candi_joint_angle_list.append([theta1, theta2, theta3-0.1, theta4, theta6])
		return candi_joint_angle_list
