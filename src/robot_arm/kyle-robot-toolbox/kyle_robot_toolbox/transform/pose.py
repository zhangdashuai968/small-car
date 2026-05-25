'''
位姿描述 
----------------------------------------------
@作者: 阿凯爱玩机器人
@QQ: 244561792
@微信: xingshunkai
@邮箱: xingshunkai@qq.com
@网址: deepsenserobot.com
@B站: "阿凯爱玩机器人"
'''
import math
import numpy as np
# 自定义库
from .transform import Transform
from .quaternion import Quaternion

class Pose:
	'''位姿
	注: 平移向量, 统一使用单位m
	'''
	# 坐标
	x = 0
	y = 0
	z = 0
	# 旋转矩阵
	rmat = np.eye(3)
	# 欧拉角
	roll = 0
	pitch = 0.0
	yaw = 0.0
	
	def set_position(self, x, y, z, unit="m"):
		'''设置位置'''
		if unit == "m":
			self.x = x
			self.y = y
			self.z = z
		elif unit == "mm":
			self.x = 0.001 * x
			self.y = 0.001 * y
			self.z = 0.001 * z
	
	def get_position(self, unit="m"):
		'''获取位置'''
		if unit == "m":
			return [self.x, self.y, self.z]
		elif unit == "mm":
			return [v*1000.0 for v in [self.x, self.y, self.z]]

	def set_euler_angle(self, roll, pitch, yaw, unit="rad"):
		'''设置欧拉角'''
		if unit == "rad":
			# 赋值欧拉角
			self.roll = roll
			self.pitch = pitch
			self.yaw = yaw
		elif unit == "deg":
			self.roll = np.radians(roll)
			self.pitch = np.radians(pitch)
			self.yaw = np.radians(yaw)
		# 更新旋转矩阵
		self.rmat = Transform.euler2rmat(\
	  		roll=self.roll, pitch=self.pitch, yaw=self.yaw)
	
	def get_euler_angle(self, unit="rad"):
		'''获取欧拉角'''
		rpy_rad = [self.roll, self.pitch, self.yaw]
		if unit == "rad":
			return rpy_rad
		elif unit == "deg":
			return np.degrees(rpy_rad)
	
	def set_rotation_matrix(self, rmat):
		'''设置旋转矩阵'''
		self.rmat = np.copy(rmat)
		# 同步更新欧拉角
		self.roll, self.pitch, self.yaw = Transform.rmat2euler(rmat)[0]
	
	def get_rotation_matrix(self):
		'''获取旋转矩阵'''
		return Transform.euler2rmat(\
	  		roll=self.roll, pitch=self.pitch, yaw=self.yaw)
	
	def set_rotation_vector(self, n, theta=None):
		'''设置旋转向量'''
		rmat = Transform.rvect2rmat(n, theta)
		self.set_rotation_matrix(rmat)
	
	def get_rotation_vector(self):
		'''获取旋转向量'''
		return Transform.rmat2rvect(self.get_rotation_matrix())
	
	def set_transform_matrix(self, tmat, unit="m"):
		'''设置变换矩阵'''
		x, y, z = tmat[:3, 3].reshape(-1)
		self.set_position(x, y, z, unit=unit)
		rmat = tmat[:3, :3]
		self.set_rotation_matrix(rmat)
	
	def get_transform_matrix(self, unit="m"):
		'''获取变换矩阵'''
		x, y, z = self.get_position(unit=unit)
		tmat = np.float64(np.eye(4))
		tmat[0,3] = x
		tmat[1,3] = y
		tmat[2,3] = z
		tmat[:3, :3] = self.rmat
		return tmat
	
	def set_quaternion(self, q):
		'''设置四元数'''
		self.set_rotation_matrix(q.to_rmat())
	
	def get_quaternion(self):
		'''获取当前的四元数'''
		q = Quaternion()
		q.from_rmat(self.rmat)
		return q
		
	def distance(self, pose, unit="m"):
		'''返回笛卡尔空间下的距离'''
		x1, y1, z1 = self.get_position(unit=unit)
		x2, y2, z2 = pose.get_position(unit=unit)
		return math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
	
	def from_bullet_pose(self, posi, q_xyzw):
		'''从Bullet位姿描述中构造Pose'''
		# 位置单位的转换
		self.set_position(*posi, unit="m")
		# 创建四元数对象
		q = Quaternion()
		q.from_xyzw(*q_xyzw)
		self.set_quaternion(q)
	
	def inverse(self):
		'''逆变换'''
		T = self.get_transform_matrix(unit="m")
		T_inv = Transform.inverse(T)
		pose_inv = Pose()
		pose_inv.set_transform_matrix(T_inv, unit="m")
		return pose_inv

	@staticmethod
	def tvect_mean(tvect_list, weights=None):
		'''旋转向量求均值
		[参数]
		@tvect_list (list / ndarray):
			平移向量列表，N*3的ndarray矩阵或list
		@weights:
			每个样本所占的比重
  		'''
		# 获取四元数的个数
		t_num = len(tvect_list)
		# 构造权重
		if weights is None:
			# 构造同等权重的列表
			weight_mean = 1.0 / t_num
			weights = [weight_mean]*t_num
		tvect_sum = np.zeros(3)
		for i in range(t_num):
			tvect_cur = np.float32(tvect_list[i])
			tvect_sum += weights[i]*tvect_cur
		tvect_sum /= np.sum(weights)
		# 返回平移向量均值
		return tvect_sum
	
	

	@staticmethod
	def tmat_mean(T_list, weights=None):
		'''对位姿(坐标变换矩阵T)求均值

		[参数]
		@T_list (list / ndarray):
			空间变换列表，N*4*4的ndarray矩阵或list
		@weights:
			每个样本所占的比重
		'''
		# 获取四元数的个数
		t_num = len(T_list)
		
		if weights is None:
			# 构造同等权重的列表
			weight_mean = 1.0 / t_num
			weights = [weight_mean]*t_num

		# 四元数列表
		q_list = []
		# 平移向量列表
		tvect_list = []
		for i in range(t_num):
			T_cur = np.float32(T_list[i])
			# 旋转矩阵 
			rmat = T_cur[:3, :3]
			# 得到四元数
			q_cur = Quaternion()
			q_cur.from_rmat(rmat)
			q_list.append(q_cur)
			# 添加平移向量
			tvect_list.append(T_cur[:3, 3])
		# 求解姿态均值
		q_mean = Quaternion.mean(q_list, \
      					weights=weights,\
               			dtype=Quaternion)
		rmat_mean = q_mean.to_rmat()
		# 求解平移向量均值 
		tvect_mean = Pose.tvect_mean(tvect_list,\
      						weights=weights)
		# 构造空间变换
		T_mean = np.eye(4)
		T_mean[:3, :3] = rmat_mean
		T_mean[:3, 3] = tvect_mean

		return T_mean
	
	@staticmethod
	def mean(pose_list, weights=None):
		'''位姿均值
		[参数]
		@T_list (list / ndarray):
			空间变换列表，N*4*4的ndarray矩阵或list
		@weights:
			每个样本所占的比重
  		'''
		if type(pose_list[0]) == Pose:
			T_list = [p.get_transform_matrix() for p in pose_list]
		else:
			T_list = pose_list
		return Pose.tmat_mean(T_list, weights=weights)

	def __str__(self):
		params = [self.x*1000.0, self.y*1000.0, self.z*1000.0,\
			np.degrees(self.roll), np.degrees(self.pitch), np.degrees(self.yaw)]
		return "Pose x={:.1f} mm, y={:.1f} mm, z={:.1f} mm, roll={:.1f}, pitch={:.1f}, yaw={:.1f}".format(*params)
