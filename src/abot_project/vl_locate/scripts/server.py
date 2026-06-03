#!/usr/bin/env python3
# _*_ coding:utf-8 _*_
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
'''

from vl_locate.srv import GetObject,GetObjectResponse
import rospy
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import time
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
import json
import base64
import sys
import os
from vl_locate.msg import Detection,DetectionArray
from dashscope import MultiModalConversation
from openai import OpenAI

class VLM:
    def __init__(self):
        # 从参数服务器获取参数
        self.input_topic = rospy.get_param('~input_topic', '/new_topic')
        self.output_topic = rospy.get_param('~output_topic', '/Pose')
        self.service_name = rospy.get_param('~service_name', 'vlm_detection')
        self.image_save_path = rospy.get_param('~image_save_path', '/home/abot/catkin_ws/src/abot_project/vl_locate/image/vlm_now.jpg')
        self.font_path = rospy.get_param('~font_path', '/home/abot/catkin_ws/src/abot_project/vl_locate/config/SimHei.ttf')
        self.font_size = rospy.get_param('~font_size', 32)
        self.api_key = rospy.get_param('~api_key', 'sk-4f51e302d219401eb0e21dc9aa7f9bbd')
        self.base_url = rospy.get_param('~base_url', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.model_name = rospy.get_param('~model_name', 'qwen-vl-max')
        self.mode = rospy.get_param('~mode', 'dashscope')
        self.bbox_color = rospy.get_param('~bbox_color', [0, 0, 255])  # BGR格式
        self.bbox_thickness = rospy.get_param('~bbox_thickness', 3)
        self.center_radius = rospy.get_param('~center_radius', 6)
        self.text_color = rospy.get_param('~text_color', [255, 0, 0])  # RGB格式
        
        # 系统提示词
        self.system_prompt = """
        检测图像中的目标，并以坐标的形式返回其位置。
        例如检测图像中的绿色方块，输出格式应为 {"label": "绿色方块", "bbox_2d": [x1, y1, x2, y2]}。
        只回复json本身即可，不要回复其它内容

        我现在的指令是：
        """
        
        # 初始化订阅者、发布者和服务
        self.sub = rospy.Subscriber(self.input_topic, ROSImage, self.top_view_shot)
        self.detection_pub = rospy.Publisher(self.output_topic, DetectionArray, queue_size=10)
        
        # 加载字体
        try:
            self.font = ImageFont.truetype(self.font_path, self.font_size)
        except:
            rospy.logwarn(f"无法加载字体文件 {self.font_path}，使用默认字体")
            self.font = ImageFont.load_default()
        
        # 创建服务
        self.service = rospy.Service(self.service_name, GetObject, self.handle_vlm_detection)
        
        rospy.loginfo(f"VLM检测节点已启动")
        rospy.loginfo(f"输入话题: {self.input_topic}")
        rospy.loginfo(f"输出话题: {self.output_topic}")
        rospy.loginfo(f"服务名称: {self.service_name}")
        rospy.loginfo(f"模式: {self.mode}")

    def encode_image(self, image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    def imgmsg_to_cv2(self, img_msg):
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                                 dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian != sys.byteorder == 'little':
            image_opencv = image_opencv.byteswap().newbyteorder()
        if img_msg.encoding == "rgb8":
            image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
        elif img_msg.encoding == "mono8":
            image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
        elif img_msg.encoding != "bgr8":
            rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
            return None
        return image_opencv

    def top_view_shot(self, image_msg):
        self.img_bgr = self.imgmsg_to_cv2(image_msg)

    def yi_vision_api(self, prompt='检测图中红色方块'):
        cv2.imwrite(self.image_save_path, self.img_bgr)
        base64_image = self.encode_image(self.image_save_path)
        
        if self.mode == "openai":
            client = OpenAI(
                api_key=self.api_key,
                base_url=self.base_url,
            )
            response = client.chat.completions.create(
                model=self.model_name, 
                messages=[{"role": "user","content": [
                {"type": "image_url","image_url":f"data:image/jpeg;base64,{base64_image}"},
                {"type": "text", "text": self.system_prompt + prompt}
                ]}]
                )
            print(response.choices[0].message.content)
            content = response.choices[0].message.content
        elif self.mode == "dashscope":
            messages = [
                {
                    "role": "user",
                    "content": [
                        {"image": self.image_save_path},
                        {"text": self.system_prompt + prompt}
                    ]
                }
            ]
            response = MultiModalConversation.call(
                api_key=self.api_key,
                model=self.model_name, 
                messages=messages
            )
            content = response["output"]["choices"][0]["message"].content[0]["text"]
  
        # 去除Markdown代码块标记（如果存在）
        if content.startswith("```json") and content.endswith("```"):
            content = content[7:-3].strip()
        
        try:
            result = json.loads(content)
            # 如果返回的是列表，取第一个元素
            if isinstance(result, list):
                result = result[0]
            rospy.loginfo('大模型调用成功！')
            return result
        except json.JSONDecodeError as e:
            rospy.logerr(f"JSON解析失败: {e}")
            rospy.logerr(f"返回内容: {content}")
            return None

    def post_processing_viz(self, result, img_path):
        img_bgr = cv2.imread(img_path)
        img_h, img_w = img_bgr.shape[:2]
        
        START_NAME = result['label']
        bbox_2d = result['bbox_2d']
        rospy.loginfo(f"检测到目标: {START_NAME}, 坐标: {bbox_2d}")
        
        x_min, y_min, x_max, y_max = map(int, bbox_2d)
        center_x = int((x_min + x_max) / 2)
        center_y = int((y_min + y_max) / 2)
        
        # 可视化
        img_bgr = cv2.rectangle(img_bgr, (x_min, y_min), (x_max, y_max), 
                               tuple(self.bbox_color), self.bbox_thickness)
        img_bgr = cv2.circle(img_bgr, (center_x, center_y), 
                            self.center_radius, tuple(self.bbox_color), -1)
        
        # 写中文标签
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img_rgb)
        draw = ImageDraw.Draw(img_pil)
        draw.text((x_min, y_min-self.font_size), START_NAME, 
                  font=self.font, fill=tuple(self.text_color))
        img_bgr = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
        
        cv2.imwrite(self.image_save_path, img_bgr)
        return center_x, center_y

    def vlm_detection(self):
        result = self.yi_vision_api('检测图中的{}'.format(self.object_name))
        if not isinstance(result, dict):
            rospy.logerr("返回结果格式错误！")
            return None
        
        X_CENTER, Y_CENTER = self.post_processing_viz(result, self.image_save_path)
        
        # 发布位姿信息  创建单个Detection消息
        pose_msg = Detection()
        pose_msg.class_name = self.object_name
        pose_msg.x_center = X_CENTER
        pose_msg.y_center = Y_CENTER
        # 创建DetectionArray消息并添加单个检测结果
        detection_array_msg = DetectionArray()
        detection_array_msg.detections.append(pose_msg)
        self.detection_pub.publish(detection_array_msg)
        return (X_CENTER, Y_CENTER)

    def handle_vlm_detection(self, req):
        self.object_name = str(req.object_name)
        result = self.vlm_detection()
        if result:
            return GetObjectResponse(success=True, message=f"检测到目标坐标：{result}")
        else:
            return GetObjectResponse(success=False, message="检测失败")

def main():
    rospy.init_node('vlm_detection_node', anonymous=True)
    vlm_node = VLM()
    rospy.loginfo("VL检测服务已启动，等待请求...")
    rospy.spin()

if __name__ == '__main__':
    main()
