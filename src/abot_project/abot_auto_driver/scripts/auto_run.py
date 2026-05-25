#!/usr/bin/env python
# coding=utf-8

import rospy
import os
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from abot_auto_driver.msg import DriverMode

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        # 订阅usb摄像头
        self.hue_black = (0,0,0,180,255,46)# black
        self.hue_red = (0,50,60,14,255,255)# red
        self.hue_blue = (100,43,46,124,255,255)# blue
        self.hue_green= (35,43,46,77,255,255)# green
        self.hue_yellow = (15,70,50,30,255,255)# yellow

        self.flag = Int8()
        self.outside = Int8()
        self.flag.data = 1
        self.outside.data = 0


        self.last_erro=0
        self.mask_x_left = 1.0000
        self.mask_x_right = 1.0000
        self.mask_y_top = 1.0000
        self.mask_y_bot = 1.0000
        self.center_target = 0.5
        self.vel_x = 0.0000
        self.vel_y_P = 0.0000
        self.vel_y_D = 0.0000
        self.vel_z_P = 0.0000
        self.vel_z_D = 0.0000
        self.state = 0
        self.state_last = 0

        self.twist = Twist()

        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw/')
        self.x = rospy.get_param("~servo_x", 60)

        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)

        self.driver_mode_sub = rospy.Subscriber("/driver_mode", DriverMode, self.driver_mode_callback)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.start_flag_pub = rospy.Publisher("/start_flag", Int8, queue_size=1)
        self.go_outside_pub = rospy.Publisher("/outside_flag", Int8, queue_size=1)

        os.system("rostopic pub -1 servo riki_msgs/Servo -- '%d' '90'" %(self.x))

    def image_callback(self, msg):
		
        image1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.resize(image1, (320,240), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        
        lowerbH = self.hue_yellow[0]
        lowerbS = self.hue_yellow[1]
        lowerbV = self.hue_yellow[2]
        upperbH = self.hue_yellow[3]
        upperbS = self.hue_yellow[4]
        upperbV = self.hue_yellow[5]

        mask1 = cv2.inRange(hsv,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
        mask2 = cv2.inRange(hsv,(lowerbH+165,lowerbS,lowerbV),(upperbH+170,upperbS,upperbV))
        mask = cv2.bitwise_or(mask1, mask2)

        h, w, d = image.shape       
        search_top = h*self.mask_y_top
        search_bot = h*self.mask_y_bot
        mask[0:int(search_top), 0:w] = 0
        mask[int(search_bot):h, 0:w] = 0
        mask[0:h, 0:int(self.mask_x_left*w)] = 0
        mask[0:h, int(self.mask_x_right*w):w] = 0
        
        # 计算mask图像的重心，即几何中心
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)

            if cv2.circle:
            # 计算图像中心线和目标指示线中心的距离
                erro = cx - w*self.center_target
                d_erro=erro-self.last_erro
                
                self.twist.linear.x = self.vel_x
                # rospy.loginfo("Point cx is %d "%cx)  
                # rospy.loginfo("Point cy is %d"%cy)
                # rospy.loginfo("Point outside is %d"%self.outside.data) 
                # rospy.loginfo("point state_last is %d"%state_last)
                # rospy.loginfo("point state is %d"%state)
                # rospy.loginfo("Point HSV is %s"%hsv[int(hsv.shape[0]/2),int(hsv.shape[1]/2)]) 
                if erro!=0 and self.state == self.state_last != 0:
                    self.twist.angular.z = -float(erro)*self.vel_z_P-float(d_erro)*self.vel_z_D
                    if self.twist.linear.y > 0.2:
						self.twist.linear.y = 0.2
                    if cx >310 and cy>170 and self.state == self.state_last == 4:
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                        sleep(0.5)
                    if cx <250 and cx>235 and cy>144 and cy<160 and self.state == self.state_last == 4:
                        self.outside.data = 1
                        self.go_outside_pub.publish(self.outside)
                        
                elif erro == 0 and self.state == self.state_last != 0:
                    self.twist.angular.z = 0
                    self.twist.linear.y = 0 
                elif self.state != self.state_last:
                    if self.state_last == 0:
                        self.state_last = self.state
                    else:
                        self.state_last = self.state
                last_erro=erro
        else:
            self.twist.angular.z = 0
        if self.state > 0:
            self.cmd_vel_pub.publish(self.twist)
            if self.state == 1:
                cv2.putText(mask, "out_way", (0,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255),1)
            elif self.state == 4:
                cv2.putText(mask, "in_way", (0,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255),1)
        else :
            cv2.putText(mask, "no_line_init", (0,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255),1)

        cv2.imshow("watch_line", mask)
        cv2.waitKey(1)
        self.start_flag_pub.publish(self.flag)


    def driver_mode_callback(self, msg): 
        self.mask_x_left = msg.mask_x_left
        self.mask_x_right = msg.mask_x_right
        self.mask_y_top = msg.mask_y_top
        self.mask_y_bot = msg.mask_y_bot
        self.center_target = msg.center_target
        self.vel_x = msg.vel_x
        self.vel_y_P = msg.vel_y_P
        self.vel_y_D = msg.vel_y_D
        self.vel_z_P = msg.vel_z_P
        self.vel_z_D = msg.vel_z_D
        self.state = msg.en

if __name__ == '__main__':
    try:
        rospy.init_node("line follower")
        follower = Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




