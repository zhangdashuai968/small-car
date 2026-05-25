import time
import numpy as np
import cv2
# 阿凯机器人工具箱
from kyle_robot_toolbox.camera import USBCamera

def main(argv):
	'''调整相机参数, 预览图像'''
	img_cnt = FLAGS.img_cnt
	# 创建相机对象
	camera = USBCamera(config_path="./config/usb_camera")
	# 初始相机
	capture = camera.init_video_stream()
	
	if FLAGS.rm_distortion:
		# 载入标定数据
		camera.load_cam_calib_data()
	# 创建一个名字叫做 “image_win” 的窗口
	win_name = 'image_win'
	cv2.namedWindow(win_name,flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
	
	fps = 40 # 设定一个初始值
	while True:
		start = time.time()
		ret, image = capture.read()
		
		if not ret:
			logging.error('图像获取失败')
			break
		if FLAGS.rm_distortion:
			# 图像去除畸变
			image = camera.remove_distortion(image)
		
		# 创建画布
		canvas = np.copy(image)
		# 添加帮助信息
		cv2.putText(canvas, text='S:Save Image',\
			 	org=(50, camera.config['img_height']-100), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
				fontScale=1, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))
		cv2.putText(canvas, text='Q: Quit',\
			 	org=(50, camera.config['img_height']-50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
				fontScale=1, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))

		# 在画布上添加帧率的信息
		cv2.putText(canvas, text='FPS: {}'.format(fps),\
			 	org=(50, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
				fontScale=1, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))
		# 更新窗口“image_win”中的图片
		cv2.imshow('image_win', canvas)

		end = time.time()
		fps = int(0.6*fps +  0.4*1/(end-start))
		# fps = 1
		key = cv2.waitKey(1)
		
		if key == ord('q'):
			# 如果按键为q 代表quit 退出程序
			break
		elif key == ord('s'):
			# s键代表保存数据
			cv2.imwrite('{}/{}.png'.format(FLAGS.img_path, img_cnt), image)
			logging.info("截图，并保存在  {}/{}.png".format(FLAGS.img_path, img_cnt))
			img_cnt += 1
	
	# 关闭摄像头
	capture.release()
	# 销毁所有的窗口
	cv2.destroyAllWindows()

if __name__ == '__main__':
	import logging
	import sys
	from absl import app
	from absl import flags

	# 设置日志等级
	logging.basicConfig(level=logging.INFO)

	# 定义参数
	FLAGS = flags.FLAGS
	flags.DEFINE_integer('img_cnt', 0, '图像计数的起始数值')
	flags.DEFINE_string('img_path', 'data/usb_camera/image_raw', '图像的保存地址')
	flags.DEFINE_boolean('rm_distortion', False, '载入相机标定数据, 去除图像畸变')
	
	# 运行主程序
	app.run(main)