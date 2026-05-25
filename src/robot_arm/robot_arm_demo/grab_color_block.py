'''
抓取工作台上的物块
----------------------------------------------
@作者: 阿凯爱玩机器人
@微信: xingshunkai
@邮箱: xingshunkai@qq.com
@B站: https://space.bilibili.com/40344504
'''
import math
import time
import cv2
import numpy as np

# 阿凯机器人工具箱
# - USB相机
from kyle_robot_toolbox.camera import USBCamera
# - 五自由度机械臂
from kyle_robot_toolbox.robot_arm.arm4dof_uservo import Arm4DoFUServo
# - 五自由度机械臂应用
from kyle_robot_toolbox.robot_arm.arm4dof_uservo import Arm4DoFApplication
# 自定义
# - 图像像素转工作台坐标
from image2workspace import Image2Workspace
# - 色块检测
from color_block_detector import ColorBlockDetector

#####################################
## 物块抓取的配置文件
#####################################
# - 物块像素尺寸最小值
cubic_min_size = 150
# - 物块像素尺寸最大值
cubic_max_size = 400
# - 机械臂抓取位姿偏移量
arm_offset_xy = 0.0 # XY平面上的偏移量
arm_offset_z = 0.0 # Z轴上的偏移量
# - 抓取阈值
# 	在某个位置检测到多少次就进行抓取
cubic_count_threshold = 5
# - 是否只抓取工作区内的物块
# 	True: 只抓取工作区内的物块
#   False: 工作区外的也抓
grab_cubic_in_ws = False
# 要抓取的色块颜色
color_names = ['blue']
#####################################

# 创建机械臂
arm_config_folder = "./config"
arm = Arm4DoFUServo(config_folder=arm_config_folder)
# 运动到图像拍摄点
arm.set_tool_pose(pose_name="capture_image")
# 创建机械臂APP
arm_app = Arm4DoFApplication(arm, config_folder=arm_config_folder)
# 创建摄像头
camera = USBCamera(config_path="config/usb_camera")
# 初始相机
camera.init_video_stream()
# 载入标定数据
camera.load_cam_calib_data()
# 色块识别器
color_block_detector = ColorBlockDetector(color_names)
# 像素到工作台坐标映射器
img2ws = Image2Workspace(camera)
# 机械臂物块抓取
# Key: 位置编码
# Value: (颜色名称, 计数)
candi_workpiece_dict = {} # 候选工件字典
def candi_workpiece_encoding(wx, wy, delta=20):
	'''工件坐标编码, 坐标单位mm'''
	return int(wx/delta) + int(wy/delta)*1000

def justify_workspace_posi(wx, wy, wz):
	'''调整工作空间坐标'''
	theta = math.atan2(wy, wx)
	r = math.sqrt(wx*wx + wy*wy)
	new_r = r + arm_offset_xy
	new_wx = new_r * math.cos(theta)
	new_wy = new_r * math.sin(theta) 
	new_wz = wz + arm_offset_z
	return new_wx, new_wy, new_wz

try:
	while True:
		# 开始时间
		t_start = time.time()
		img_bgr = camera.read_color_img()
		# 去除畸变
		img_bgr = camera.remove_distortion(img_bgr)
		# 识别色块
		canvas = np.copy(img_bgr)
		for color_name in color_names:
			rect_list, canvas = color_block_detector.color_clock_rect(\
				img_bgr, color_name, canvas=canvas)
			# 遍历所有的ROI框
			for rect in rect_list:
				x, y, w, h = rect
				# 过滤物块
				if w < cubic_min_size or w > cubic_max_size:
					print(f"像素宽度 = {w}, 不满足尺寸要求")
					continue
				if h < cubic_min_size or h > cubic_max_size:
					print(f"像素高度 = {h}, 不满足尺寸要求")
					continue
				# 计算物体像素中心坐标
				# 计算物体中心坐标
				if h > w:
					px = x + w/2
					py = y + h - 0.8*(w/2)
				else:
					px = x + w/2
					py = y + h/2
				# 绘制中心点
				cv2.circle(canvas, (int(px), int(py)), \
        			10, (255, 0, 255), thickness=-1)
				
    			# 将图像坐标系转换为工作台坐标系
				wx, wy = img2ws.img2workspace(px, py)
				# 绘制帧率
				cv2.putText(canvas, text=f"wx {wx:.1f} wy {wy:.1f}",\
					org=(int(px), int(py+10)), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
					fontScale=0.8, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))
				
  
				# print(f"wx = {wx} wy = {wy}")
				# 判断是否在工作区内
				if grab_cubic_in_ws:
					if abs(wx) > img2ws.x0 or abs(wy) > img2ws.y0:
						# 在工作区格子外，不处理
						print(f"超出工作区范围: wx = {wx} wy = {wy}")
						continue 
				# 对位置进行编码
				code = candi_workpiece_encoding(wx, wy)
				if code not in candi_workpiece_dict:
					candi_workpiece_dict[code] = [color_name, 1]
				else:
					candi_workpiece_dict[code][0] = color_name
					candi_workpiece_dict[code][1] += 1
				# print(f"纳入统计: wx = {wx} wy = {wy} 编码 = {code}")
				if(candi_workpiece_dict[code][1] >= cubic_count_threshold):
					# 抓取工件
					print("- 抓取工件")
					print(f"物块颜色 {color_name} wx={wx:.1f} wy={wy:.1f} mm")
					wx2, wy2, wz2 = justify_workspace_posi(wx, wy, arm_app.object_height)
					arm_app.grab_cubic(wx2, wy2, wz2)
					# 清空投票字典
					candi_workpiece_dict = {}
					break
		# 统计帧率
		t_end = time.time()
		t_pass = t_end - t_start
		fps = int(1/t_pass)

		# 绘制帧率
		cv2.putText(canvas, text=f"FPS:{fps}",\
			org=(20, 20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
			fontScale=0.8, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))
		
		cv2.imshow("canvas", canvas)
		key = cv2.waitKey(1)
		if key == ord('q'):
			break
except KeyboardInterrupt:
	print("按键中断")
finally:
	# 关闭所有的窗口
	cv2.destroyAllWindows()
	# 释放相机
	camera.release()
	# 机械臂卸力
	arm.set_damping(0)
