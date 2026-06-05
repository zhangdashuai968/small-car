#!/usr/bin/env python
# _*_ coding:utf-8 _*_
import rospy
import numpy as np
import cv2
import sys
from sensor_msgs.msg import Image

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    if img_msg.encoding == "rgb8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "mono8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
    elif img_msg.encoding != "bgr8":
        rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
        return None
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

_frame_count = 0
_IMG_W, _IMG_H = 640, 480
_ROI_X1 = int(_IMG_W * 0.43)
_ROI_Y1 = int(_IMG_H * 0.53)
_ROI_X2 = int(_IMG_W * 0.75)
_ROI_Y2 = int(_IMG_H * 0.78)
_ROI_W = _ROI_X2 - _ROI_X1
_ROI_H = _ROI_Y2 - _ROI_Y1
_SCALE = min(_IMG_W / float(_ROI_W), _IMG_H / float(_ROI_H))
_NEW_W = int(_ROI_W * _SCALE)
_NEW_H = int(_ROI_H * _SCALE)
_START_X = (_IMG_W - _NEW_W) // 2
_START_Y = (_IMG_H - _NEW_H) // 2

def image_callback(img_msg):
    global _frame_count
    _frame_count += 1
    if _frame_count % 2 != 0:
        return

    cv_image = imgmsg_to_cv2(img_msg)
    if cv_image is None:
        return

    # CPU: crop ROI (must copy for contiguous GPU upload)
    roi = cv_image[_ROI_Y1:_ROI_Y2, _ROI_X1:_ROI_X2].copy()

    # GPU: resize with INTER_CUBIC
    gpu_src = cv2.cuda_GpuMat()
    gpu_src.upload(roi)
    gpu_dst = cv2.cuda.resize(gpu_src, (_NEW_W, _NEW_H), interpolation=cv2.INTER_CUBIC)
    zoomed_roi = gpu_dst.download()

    # CPU: center placement
    zoomed = np.zeros((_IMG_H, _IMG_W, 3), dtype=np.uint8)
    zoomed[_START_Y:_START_Y+_NEW_H, _START_X:_START_X+_NEW_W] = zoomed_roi

    pub.publish(cv2_to_imgmsg(zoomed))

if __name__ == '__main__':
    rospy.init_node('image_zoomer', anonymous=True)
    pub = rospy.Publisher('/new_topic', Image, queue_size=10)
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
    rospy.spin()
