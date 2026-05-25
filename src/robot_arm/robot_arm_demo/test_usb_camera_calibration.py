from kyle_robot_toolbox.camera_calibration import CameraCalibration

def main(argv):
	# 创建相机标定对象
	cc = CameraCalibration(config_path=FLAGS.config_path, \
			img_folder=FLAGS.img_folder, \
			save_path=FLAGS.save_path)
	# 打印相机标定数据
	cc.print_parameter()
	# 相机标定数据序列化
	cc.dump_camera_info()
	
if __name__ == "__main__":
	import logging
	import sys
	from absl import app
	from absl import flags

	# 定义参数
	FLAGS = flags.FLAGS
	# 定义相机标定配置文件路径
	flags.DEFINE_string('config_path', \
     	'config/usb_camera/caliboard.yaml', '相机标定配置文件路径')
	flags.DEFINE_string('img_folder', \
     	'data/usb_camera/caliboard', '标定板图像路径')
	flags.DEFINE_string('save_path', \
     	'config/usb_camera', '相机标定数据保存路径')
	# 运行主程序
	app.run(main)
