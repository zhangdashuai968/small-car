'''
相机抽象类
----------------------------------------------
@作者: 阿凯爱玩机器人
@QQ: 244561792
@微信: xingshunkai
@邮箱: xingshunkai@qq.com
@网址: deepsenserobot.com
@B站: "阿凯爱玩机器人"
'''
import os
import yaml
import numpy as np
import cv2
#import open3d as o3d

class Camera:
	'''相机对象'''
	# 深度相机拍摄范围
	min_distance = 0
	max_distance = 2000
	def __init__(self, img_width=None, img_height=None, \
			config_path=None, load_calib_data=False):
		'''初始化相机'''
		if config_path is not None:
			self.config_path = config_path
			# 载入相机的配置文件
			camera_config_path = os.path.join(config_path, "camera.yaml")
			with open(camera_config_path, 'r', encoding='utf-8') as f:
				self.config = yaml.load(f.read(), Loader=yaml.SafeLoader)
			# 获取图像的宽度与高度
			self.img_width = self.config["img_width"]
			self.img_height = self.config["img_height"]
		else:
			self.config_path = None
			# - 设置图像宽度
			self.img_width = img_width
			# - 设置图像高度
			self.img_height = img_height
		# # 载入标定数据
		# if load_calib_data:
		# 	self.load_cam_calib_data()

	def set_parameter(self, intrinsic, distortion=None):
		'''设置相机参数'''
		# 赋值相机内参
		self.intrinsic = intrinsic
		# 赋值畸变系数
		if distortion is not None:
			self.distortion = distortion
		else:
			self.distortion = np.ndarray([0, 0, 0, 0, 0]).astype("float64")
		# TODO 计算去除畸变后的内参矩阵
		self.intrinsic_new = intrinsic
		self.distortion_new = np.ndarray([0, 0, 0, 0, 0]).astype("float64")
		# 根据相机标定参数
		# 提取图像中心(cx, cy)与焦距f(单位：像素)
		self.fx = self.intrinsic[0, 0]
		self.fy = self.intrinsic[1, 1]
		self.f = (self.fx + self.fy) / 2
		# 图像中心的坐标
		self.cx = self.intrinsic[0, 2]
		self.cy = self.intrinsic[1, 2]
		# 生成视场角等相关参数
		self.alpha1 = np.arctan(self.cy/self.f)
		self.alpha2 = np.arctan((self.img_height-self.cy)/self.f)
		self.beta1 = np.arctan(self.cx/self.f)
		self.beta2 = np.arctan((self.img_width-self.cx)/self.f)
		# 创建Open3D的内参矩阵对象
		#self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
		#	self.img_width, self.img_height, \
		#	self.fx, self.fy, self.cx, self.cy)
		
	def load_cam_calib_data(self, config_path=None):
		'''载入相机标定数据'''
		if config_path is None:
			config_path = self.config_path
		
		def get_path(fname):
			return os.path.join(config_path, fname)
		
		# 读取标定参数
		# 获取摄像头内参
		self.intrinsic = np.loadtxt(get_path('M_intrisic.txt'), delimiter=',')
  		# 获取摄像头的畸变系数
		self.distortion = np.loadtxt(get_path('distor_coeff.txt'), delimiter=',')
		# 去除畸变后的相机内参与畸变系数
		self.intrinsic_new = np.loadtxt(get_path('M_intrisic_new.txt'), delimiter=',')
		self.distortion_new = np.ndarray([0, 0, 0, 0, 0]).astype("float64")
  		# x轴的映射
		# self.remap_x = np.loadtxt('config/remap_x.txt', delimiter=',').astype("float32")
		self.remap_x = np.load(get_path("remap_x.npy"))
  		# y轴映射
		# self.remap_y = np.loadtxt('config/remap_y.txt', delimiter=',').astype("float32")
		self.remap_y = np.load(get_path("remap_y.npy"))
		# 根据相机标定参数
		# 提取图像中心(cx, cy)与焦距f(单位：像素)
		self.fx = self.intrinsic_new[0, 0]
		self.fy = self.intrinsic_new[1, 1]
		self.f = (self.fx + self.fy) / 2
		
		# 图像中心的坐标
		self.cx = self.intrinsic_new[0, 2]
		self.cy = self.intrinsic_new[1, 2]
		# 生成视场角等相关参数
		self.alpha1 = np.arctan(self.cy/self.f)
		self.alpha2 = np.arctan((self.img_height-self.cy)/self.f)
		self.beta1 = np.arctan(self.cx/self.f)
		self.beta2 = np.arctan((self.img_width-self.cx)/self.f)
		# 创建Open3D的内参矩阵对象
		#self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
		#	self.img_width, self.img_height, \
		#	self.fx, self.fy, self.cx, self.cy)
	
	def remove_distortion(self, image):
		'''图像去除畸变'''
		if hasattr(self, 'remap_x') and hasattr(self, 'remap_y'):
			return cv2.remap(image, self.remap_x, self.remap_y, cv2.INTER_LINEAR)
		else:
			return image
	
	def cam_point3d2pixel2d(self, cam_x, cam_y, cam_z, intrinsic=None):
		'''将3D点投影到像素平面'''
		if intrinsic is not None:
			fx = intrinsic[0, 0]
			fy = intrinsic[1, 1]
			cx = intrinsic[0, 2]
			cy = intrinsic[1, 2]
		else:
			fx = self.fx
			fy = self.fy
			cx = self.cx
			cy = self.cy
		cam_z_inv = 1 / cam_z
		x_uz = cam_x * cam_z_inv
		y_uz = cam_y * cam_z_inv
		u = np.int32(fx * x_uz + cx)
		v = np.int32(fy * y_uz + cy)

		return u, v
	
	def depth_pixel2cam_point3d(self, px, py, \
			depth_image=None,\
			depth_value=None, intrinsic=None):
		'''深度像素转换为相机坐标系下三维坐标'''
		# 获得深度
		if depth_image is not None:
			z_cam = depth_image[py, px]
		elif depth_value is not None:
			z_cam = depth_value
		if intrinsic is not None:
			fx = intrinsic[0, 0]
			fy = intrinsic[1, 1]
			cx = intrinsic[0, 2]
			cy = intrinsic[1, 2]
		else:
			fx = self.fx
			fy = self.fy
			cx = self.cx
			cy = self.cy
			
		x_cam = (px - cx) / fx * z_cam
		y_cam = (py - cy) / fy * z_cam
		return [x_cam, y_cam, z_cam]
	
	def depth_img2canvas(self, depth_img, min_distance = None,\
			max_distance = None):
		'''将深度图转换为画布'''
		if min_distance is None:
			min_distance = self.min_distance
			max_distance = self.max_distance
		# 距离阈值的单位是mm
		depth_img_cut = np.copy(depth_img).astype(np.float32)
		depth_img_cut[depth_img < min_distance] = min_distance
		depth_img_cut[depth_img > max_distance] = max_distance
		# 归一化
		depth_img_norm = np.uint8(255.0*(depth_img_cut-min_distance)/(max_distance - min_distance))
		# 转换为
		depth_colormap = cv2.applyColorMap(depth_img_norm, cv2.COLORMAP_JET)
		return depth_colormap
	
	def get_pcd(self, color_image, depth_image,\
			mask=None, min_distance=None, max_distance=None, \
       		o3d_intrinsic=None):
		'''从彩图与深度图构建Open3D格式的点云数据
  		@o3d_intrinsic: 可以指定内参
    	'''
		# 对深度图进行过滤
		# 超出这个范围内的都当作为无效距离
		depth_image_cut = np.copy(depth_image)
		# 根据Mask来筛选深度像素(点云)
		if mask is not None:
			mask_pixel_idx = (mask == 0)
			depth_image_cut[mask_pixel_idx] = 0
		if min_distance is not None:
			depth_image_cut[depth_image < min_distance] = 0
		if max_distance is not None:
			depth_image_cut[depth_image > max_distance] = 0
		# 深度的单位从mm转换为m
		depth_image_cut = np.float32(depth_image_cut) * 0.001
		# 彩图转换为RGB色彩空间
		color_image_o3d = o3d.geometry.Image(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
		depth_image_o3d = o3d.geometry.Image(depth_image_cut)
		# 装换为RGBD图
		# - depth_trunc : Depth values larger than depth_trunc gets truncated to 0. The depth values will first be scaled and then truncated.
		#   截止深度, 大于这个值的设置为0
		# - depth_scale : 深度的尺度， 先缩放然后再trunc
		depth_scale = 1
		rgbd_o3d = o3d.geometry.RGBDImage.create_from_color_and_depth(\
			color_image_o3d, depth_image_o3d, depth_scale=depth_scale, convert_rgb_to_intensity=False)
		# 将RGBD图转换为点云格式pcd
		if o3d_intrinsic is None:
			o3d_intrinsic = self.pinhole_camera_intrinsic
		pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_o3d, o3d_intrinsic)
		return pcd

	def get_pcd2(self, depth_image,\
			mask=None, min_distance=None, max_distance=None, \
	   		rgb_color=None, o3d_intrinsic=None):
		'''从深度图构建Open3D格式的点云数据
  		'''
		if rgb_color is None:
			# 给点云赋值为可视化深度图的颜色
			# color_image = np.copy(self.depth_img2canvas(depth_image), dtype=np.uint8)
			color_image = self.depth_img2canvas(depth_image)
		else:
			# 给点云赋值为同一个颜色(rgb_color)
			img_h, img_w = depth_image.shape
			color_image = np.ones((img_h, img_w, 3), dtype=np.uint8)
			color_image[:, :] = rgb_color
		# 对深度图进行过滤
		# 超出这个范围内的都当作为无效距离
		depth_image_cut = np.copy(depth_image)
		# 根据Mask来筛选深度像素(点云)
		if mask is not None:
			mask_pixel_idx = (mask == 0)
			depth_image_cut[mask_pixel_idx] = 0
		if min_distance is not None:
			depth_image_cut[depth_image < min_distance] = 0
		if max_distance is not None:
			depth_image_cut[depth_image > max_distance] = 0
		# 深度的单位从mm转换为m
		depth_image_cut = np.float32(depth_image_cut) * 0.001
		# 彩图转换为RGB色彩空间
		color_image_o3d = o3d.geometry.Image(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
		depth_image_o3d = o3d.geometry.Image(depth_image_cut)
		# 装换为RGBD图
		# - depth_trunc : Depth values larger than depth_trunc gets truncated to 0. The depth values will first be scaled and then truncated.
		#   截止深度, 大于这个值的设置为0
		# - depth_scale : 深度的尺度， 先缩放然后再trunc
		depth_scale = 1.0
		rgbd_o3d = o3d.geometry.RGBDImage.create_from_color_and_depth(\
			color_image_o3d, depth_image_o3d, depth_scale=depth_scale, convert_rgb_to_intensity=False)
		# 将RGBD图转换为点云格式pcd
		if o3d_intrinsic is None:
			o3d_intrinsic = self.pinhole_camera_intrinsic
		pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_o3d, o3d_intrinsic)
		return pcd

	def pcd2depth_image(self, pcd, intrinsic=None):
		'''PCD点云转换为深度图
    	'''
		if intrinsic is not None:
			fx = intrinsic[0, 0]
			fy = intrinsic[1, 1]
			cx = intrinsic[0, 2]
			cy = intrinsic[1, 2]
		else:
			fx = self.fx
			fy = self.fy
			cx = self.cx
			cy = self.cy
		# 构造深度图
		depth_image_align = np.zeros((self.img_height, self.img_width), dtype=np.float32)
		# 获取3D点坐标
		point3d_arr = np.asarray(pcd.points, dtype=np.float32)
		# 计算3D点对应的像素坐标
		cam_x = point3d_arr[:, 0]
		cam_y = point3d_arr[:, 1]
		cam_z = point3d_arr[:, 2]
		pixel_u, pixel_v = self.cam_point3d2pixel2d(cam_x, cam_y, cam_z, intrinsic=intrinsic)
		
		# 筛选有效的像素点
		valid_idx = np.bitwise_and(np.bitwise_and(pixel_u > 0,  pixel_u < self.img_width), \
								np.bitwise_and(pixel_v > 0,  pixel_v < self.img_height))
		pixel_u_valid = pixel_u[valid_idx]
		pixel_v_valid = pixel_v[valid_idx]

		# 填写深度, 单位m -> mm
		# TODO 如果深度图像素一样，选择近处的那个点
		depth_image_align[pixel_v_valid, pixel_u_valid] = cam_z[valid_idx] * 1000.0
		return depth_image_align

	def add_color_on_pcd(self, pcd, color_image):
		'''给PCD上色'''
		color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
		# 获取3D点列表
		point3d_arr = np.asarray(pcd.points)
		point_num  = len(point3d_arr)
		# 获取XYZ坐标
		cam_x = point3d_arr[:, 0]
		cam_y = point3d_arr[:, 1]
		cam_z = point3d_arr[:, 2]
		# 转换为相机坐标系
		pixel_u, pixel_v = self.cam_point3d2pixel2d(cam_x, cam_y, cam_z)
		valid_idx = np.bitwise_and(np.bitwise_and(pixel_u > 0,  pixel_u < self.img_width), \
      		np.bitwise_and(pixel_v > 0,  pixel_v < self.img_height))

		pixel_u_valid = pixel_u[valid_idx]
		pixel_v_valid = pixel_v[valid_idx]
		# 给点云上色
		colors = np.asarray(pcd.colors)
		colors[valid_idx] = np.float32(color_image_rgb[pixel_v_valid, pixel_u_valid])/ 255.0
		pcd.colors = o3d.utility.Vector3dVector(colors) 
		return pcd
