'''
获取工作台的无畸变图像中棋盘的9点坐标
'''
import os
import time
import cv2
import numpy as np
import pickle
import subprocess
import math
import yaml
import logging
# 阿凯机器人工具箱
from kyle_robot_toolbox.camera import USBCamera

points_num = 9
pionts_order = list(range(points_num))
points_color = [[0, 0, 255], [0, 255, 0], [255, 0, 0],\
    			[0, 0, 255], [0, 255, 0], [255, 0, 0],\
           		[0, 0, 255], [0, 255, 0], [255, 0, 0]]
points_list = []

def draw_points(canvas):
	global pionts_order
	global points_color
	global points_list
	n_point = len(points_list)
	for i in range(n_point):
		x, y = points_list[i]
		label = pionts_order[i]
		color = points_color[i]
		cv2.circle(canvas, (x,y), 8, color, -1)
		# 选择字体
		font = cv2.FONT_HERSHEY_SIMPLEX
		cv2.putText(canvas, text=f"P{label}:[{x}, {y}]", \
            org=(x+20, y), fontFace=font, fontScale=0.5, \
            thickness=1, lineType=cv2.LINE_AA, color=color)

def mouse_callback(event,x,y,flags,param):
	global points_list
	# 判断事件是否为 Left Button Double Click 
	if event == cv2.EVENT_LBUTTONDBLCLK and len(points_list) < points_num:
		print(f"添加点: x={x}  y={y}")
		points_list.append([x, y])
		if len(points_list) == points_num:
			print("完成角点采集")
			print(points_list)
			# 保存角点
			np.savetxt('config/handeye_calibration/img9points.txt', points_list, delimiter=',', fmt='%d')

# 创建相机对象
config_path = os.path.join("config", "usb_camera")
camera = USBCamera(config_path=config_path)
# 初始相机
camera.init_video_stream()
# 载入标定数据
camera.load_cam_calib_data()


# 创建一个名字叫做 “image_win” 的窗口
win_name = 'image_win'
cv2.namedWindow(win_name,flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

# 设置鼠标事件回调
cv2.setMouseCallback('image_win', mouse_callback)  

while True:
	# 获取图像
	image = camera.read_color_img()
	# 图像去除畸变
	image = camera.remove_distortion(image)	
	# 创建画布
	canvas = np.copy(image)
	# 添加帮助信息
	if len(points_list) == points_num:
		cv2.putText(canvas, text='Board Corner -> Done',\
				org=(50, camera.config['img_height']-100), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
				fontScale=1, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))
		
	cv2.putText(canvas, text='Q: Quit',\
			org=(50, camera.config['img_height']-50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
			fontScale=1, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))
	
	draw_points(canvas)
 
	# 更新窗口“image_win”中的图片
	cv2.imshow('image_win', canvas)

	key = cv2.waitKey(1)
	if key == ord('q'):
		# 如果按键为q 代表quit 退出程序
		break

# 关闭摄像头
camera.release()
# 销毁所有的窗口
cv2.destroyAllWindows()