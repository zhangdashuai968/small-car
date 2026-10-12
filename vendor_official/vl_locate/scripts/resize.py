#!/usr/bin/env python
# _*_ coding:utf-8 _*_
import rospy
import numpy as np
import cv2
import sys
from sensor_msgs.msg import Image

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")  # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)

    # If the byte order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    # Convert to BGR if the encoding is not already BGR
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
    img_msg.step = len(img_msg.data) // img_msg.height  # That double line is actually integer division, not a comment
    return img_msg

def image_callback(img_msg):
    # Convert ROS Image message to OpenCV image
    cv_image = imgmsg_to_cv2(img_msg)
    
    if cv_image is not None:
        # Get the dimensions of the image
        height, width = cv_image.shape[:2]
        
        # Define the red box region coordinates
        x1 = int(width * 0.43)
        y1 = int(height * 0.53)
        x2 = int(width * 0.75)
        y2 = int(height * 0.78)
        
        # Ensure coordinates are within image bounds
        x1 = max(x1, 0)
        y1 = max(y1, 0)
        x2 = min(x2, width)
        y2 = min(y2, height)
        
        # Extract the region of interest (ROI)
        roi = cv_image[y1:y2, x1:x2]
        
        # 方案1：保持纵横比的放大
        roi_height, roi_width = roi.shape[:2]
        
        # 计算放大倍数，保持纵横比
        scale_x = width / roi_width
        scale_y = height / roi_height
        scale = min(scale_x, scale_y)  # 使用较小的缩放比例保持纵横比
        
        # 计算新的尺寸
        new_width = int(roi_width * scale)
        new_height = int(roi_height * scale)
        
        # 使用更好的插值方法进行放大
        if scale > 2.0:
            # 对于大倍数放大，使用立方插值
            zoomed_roi = cv2.resize(roi, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
        else:
            # 对于小倍数放大，使用线性插值
            zoomed_roi = cv2.resize(roi, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        
        # 创建黑色背景图像
        zoomed_image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 计算居中位置
        start_x = (width - new_width) // 2
        start_y = (height - new_height) // 2
        
        # 将放大的ROI放置在中心
        zoomed_image[start_y:start_y+new_height, start_x:start_x+new_width] = zoomed_roi
        
        # Convert back to ROS Image message
        zoomed_img_msg = cv2_to_imgmsg(zoomed_image)
        
        # Publish the zoomed image
        pub.publish(zoomed_img_msg)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('image_zoomer', anonymous=True)
    
    # Create a publisher for the new topic
    pub = rospy.Publisher('/new_topic', Image, queue_size=10)
    
    # Subscribe to the /usb_cam/image_raw topic
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
    
    # Keep the node running
    rospy.spin()
