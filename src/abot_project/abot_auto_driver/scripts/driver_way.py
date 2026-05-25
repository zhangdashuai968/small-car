#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2, cv_bridge
import numpy
import os
import sys
from abot_auto_driver.msg import DriverMode
import threading
import time
from geometry_msgs.msg import Twist
from vision_msgs.msg import BoundingBox2D, Detection2DArray,ObjectHypothesisWithPose,Detection2D
from dynamic_reconfigure.server import Server
from abot_auto_driver.cfg import paramsConfig


class ControlDriver:
    def __init__(self):

        rospy.init_node("control_drive")

        self.outer_ring_center_target = rospy.get_param('~outer_ring_center_target', 0.26)
        self.outer_ring_vel_z_P = rospy.get_param('~outer_ring_vel_z_P', 0.01) 
        self.outer_ring_vel_z_D = rospy.get_param('~outer_ring_vel_z_D', 0.001) 

        self.Inner_ring_center_target = rospy.get_param('~Inner_ring_center_target', 0.85) 
        self.Inner_ring_vel_y_P = rospy.get_param('~Inner_ring_vel_y_P', 0.001) 
        self.Inner_ring_vel_y_D = rospy.get_param('~Inner_ring_vel_y_D', 0.003) 
        self.Inner_ring_vel_z_P = rospy.get_param('~Inner_ring_vel_z_P', 0.006) 
        self.Inner_ring_vel_z_D = rospy.get_param('~Inner_ring_vel_z_D', 0.001) 
        
        rate = rospy.Rate(100)

        self.outer_ring_image_x_left = 0
        self.outer_ring_image_x_right = 0.7
        self.outer_ring_image_y_top = 0.6
        self.outer_ring_image_y_bot = 0.85
        self.outer_ring_vel_x = 0.15
        self.outer_ring_vel_y_P = 0
        self.outer_ring_vel_y_D = 0

        self.Inner_ring_mask_x_left = 0.5
        self.Inner_ring_mask_x_right = 1
        self.Inner_ring_mask_y_top = 0.6
        self.Inner_ring_mask_y_bot = 0.85
        self.Inner_ring_vel_x = 0.1

        self.side_flag = 0
        self.stop_flag = 0
        self.start_flag = 0
        self.go_outside_flag = 0
        self.last_ring = 0
        self.last_side = 0
        self.stop_back_run = 0
        self.stop_temp = 0
        self.park_temp = 0
        self.park_flag = 0
        self.park_restart = 0
        self.lock_turn = False
        self.lock_parking = False

        self.driver_mode_pub = rospy.Publisher("/driver_mode", DriverMode, queue_size=1)

        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.object_sub = rospy.Subscriber("/abot_detect_node/detections", Detection2DArray, self.ObjectCallback)
        self.start_flag_sub = rospy.Subscriber("/start_flag", Int8, self.start_flag_callback)
        self.outside_flag_sub = rospy.Subscriber("/outside_flag", Int8, self.outside_flag_callback)
        self.park_restart_flag_sub = rospy.Subscriber("/park_restart_flag", String, self.park_restart_callback)
        
        self.dynamic_reconfigure_server = Server(paramsConfig, self.reconfigCB)
        abot_driver_way = 0 
        side = 0  
        i = 0

        drivermode_pub = DriverMode()
        decelerate = Int8()
        msg = Int8()

        while not rospy.is_shutdown():
            if self.start_flag == 1:
                if abot_driver_way == 0:  
                    side = 1
                    abot_driver_way = 1
                    self.last_ring = abot_driver_way
                    self.last_side = side
                    drivermode_pub = self.ctrl_data(abot_driver_way, side)
                    self.driver_mode_pub.publish(drivermode_pub)
                    print("outer_ring")
                    side = 0
                        
                elif abot_driver_way == 1:
                    if i % 150 == 0:
                        print("outleft")

                    if self.side_flag == 1:
                        side = 1
                        self.side_flag = 0
                    elif self.side_flag == 2:
                        side = 2
                        self.side_flag = 0
                    
                    #turn right mode
                    if side == 2:
                        print("turn right ----------------action")
                        car_stop = DriverMode()
                        self.driver_mode_pub.publish(car_stop)
                        carmove = Twist()
                        carmove.linear.x = 0.15
                        carmove.angular.z = 0
                        self.cmdvel_pub.publish(carmove)
                        self.cmdvel_pub.publish(carmove)
                        time.sleep(4.0)
                        carmove.linear.x = 0
                        carmove.angular.z = -1.5
                        self.cmdvel_pub.publish(carmove)
                        self.cmdvel_pub.publish(carmove)
                        time.sleep(1.1)
                        carmove.linear.x = 0.15
                        carmove.angular.z = 0
                        self.cmdvel_pub.publish(carmove)
                        self.cmdvel_pub.publish(carmove)
                        abot_driver_way = 2
                        last_abot_driver_way = abot_driver_way
                        self.last_side = side
                        drivermode_pub = self.ctrl_data(abot_driver_way, side)
                        self.driver_mode_pub.publish(drivermode_pub)
                        print("in_ri")
                        side = 0
                        self.side_flag = 0
                        self.lock_turn = False
                        time.sleep(2)
                        temp4 = 0

                elif abot_driver_way == 2:		
                    if i % 150 == 0:
                        print("inside")
                    #detect end line
                    if self.go_outside_flag == 1 and self.stop_flag == 0:
                        print("start_outside-----------------")
                        car_stop = DriverMode()
                        self.driver_mode_pub.publish(car_stop)
                        carmove = Twist()
                        carmove.linear.x = 0.15
                        carmove.angular.z = 0
                        self.cmdvel_pub.publish(carmove)
                        time.sleep(0.6)
                        print("go_over")
                        self.cmdvel_pub.publish(carmove)
                        carmove.linear.x = 0
                        carmove.angular.z = -1.5
                        self.cmdvel_pub.publish(carmove)
                        time.sleep(1.1)
                        print("turn_over")
                        abot_driver_way = 1
                        side = 1
                        last_abot_driver_way = abot_driver_way
                        self.last_side = side
                        drivermode_pub = self.ctrl_data(abot_driver_way, side)
                        self.driver_mode_pub.publish(drivermode_pub)
                        side = 0
                        go_outside_flag = 0
                        print("out_le")
                    if self.stop_flag == 1:
                        print("traffic stop------------------")
                        car_stop = DriverMode()
                        self.driver_mode_pub.publish(car_stop)
                        carmove = Twist()
                        carmove.linear.x = 0
                        carmove.angular.z = 0
                        self.cmdvel_pub.publish(carmove)
                        time.sleep(1)
                        if self.stop_back_run == 1:
                            print("start")
                            carmove = Twist()
                            carmove.linear.x = 0.15
                            carmove.angular.z = 0
                            self.cmdvel_pub.publish(carmove)
                            time.sleep(2)
                            drivermode_pub = self.ctrl_data(last_abot_driver_way, 2)
                            self.driver_mode_pub.publish(drivermode_pub)
                            self.stop_back_run = 0
                            self.stop_flag = 0
                            print("Inner_ring")
                    if self.park_flag == 1:
                        self.park_flag = 0
                        print("part time")
                        car_stop = DriverMode()
                        self.driver_mode_pub.publish(car_stop)
                        carmove = Twist()
                        carmove.linear.x = 0.15
                        carmove.angular.z = 0
                        self.cmdvel_pub.publish(carmove)
                        self.cmdvel_pub.publish(carmove)
                        time.sleep(2.8)
                        carmove.linear.x = 0
                        carmove.linear.y = -0.15
                        carmove.angular.z = 0
                        self.cmdvel_pub.publish(carmove)
                        self.cmdvel_pub.publish(carmove)
                        time.sleep(2.5)
                        carmove.linear.x = 0
                        carmove.linear.y = 0
                        carmove.angular.z = 0
                        self.cmdvel_pub.publish(carmove)
                        self.cmdvel_pub.publish(carmove)
                        time.sleep(1.0)
                    if self.park_restart == 1:
                        carmove = Twist()
                        carmove.linear.x = 0
                        carmove.linear.y = 0.15
                        carmove.angular.z = 0
                        self.cmdvel_pub.publish(carmove)
                        self.cmdvel_pub.publish(carmove)
                        time.sleep(2.5)
                        drivermode_pub = self.ctrl_data(last_abot_driver_way, self.last_side)
                        self.driver_mode_pub.publish(drivermode_pub)
                        self.park_restart = 0
                        self.lock_parking = False

                    i = i + 1
                    if i > 50000:
                        i = 0
                        
            else :
                 print("waiting for init")

            rate.sleep()


    def start_flag_callback(self, msg):
        self.start_flag = msg.data

    def outside_flag_callback(self, msg):
        self.go_outside_flag = msg.data

    def park_restart_callback(self, msg):
        if msg.data == "start":
            self.park_restart = 1

    def ObjectCallback(self, msg):
        traffic_id = msg.detections[-1].header.frame_id
        score = msg.detections[-1].results[-1].score
        ptx = msg.detections[-1].bbox.center.x
        pty = msg.detections[-1].bbox.center.y
        ptw = msg.detections[-1].bbox.size_x
        pth = msg.detections[-1].bbox.size_y

        if msg.detections == [] or traffic_id == "traffic_green":
            if self.stop_flag == 1:
                self.stop_temp = self.stop_temp + 1
                print("stop_temp=%d"%self.stop_temp)
                if self.stop_temp > 15:
                    self.stop_back_run = 1
                    print("stop_temp=%d"%self.stop_temp)
                    self.stop_temp = 0 
        elif traffic_id == "turn_right" and self.lock_turn is False and score > 0.9 :
            print("turn right", ptx, pty, pth*ptw)
            if ptx> 500 and ptx < 550 and pth*ptw > 1500:
                self.side_flag = 2
                self.lock_turn = True
                print("start turn right----------")
        elif traffic_id == "traffic_red":
            print("traffic:", ptx, pty, pth*ptw)
            if ptx < 110 and pth*ptw > 2800:  #pro
                self.stop_flag = 1
                self.stop_temp = 0
        #elif traffic_id == "person":
        #        self.stop_flag = 1
        #        self.stop_temp = 0
        elif traffic_id == "parking" and self.lock_parking is False and score > 0.9:
            print("parking", ptx, pty, pth*ptw)
            if pty > 72 and pty < 82 and ptw*pth > 2200 and ptw*pth < 3000:
                print("start parking-----------------------")
                self.park_flag = 1
                self.lock_parking = True

            
    def ctrl_data(self, abot_driver_way, side):
        drivermode = DriverMode()
        if abot_driver_way == 1:
            if side == 1:	#寻左线行驶
                drivermode.mask_x_left = self.outer_ring_image_x_left
                drivermode.mask_x_right = self.outer_ring_image_x_right
                drivermode.mask_y_top = self.outer_ring_image_y_top
                drivermode.mask_y_bot = self.outer_ring_image_y_bot
                drivermode.center_target = self.outer_ring_center_target
                drivermode.vel_x = self.outer_ring_vel_x
                drivermode.vel_y_P = self.outer_ring_vel_y_P
                drivermode.vel_y_D = self.outer_ring_vel_y_D
                drivermode.vel_z_P = self.outer_ring_vel_z_P
                drivermode.vel_z_D = self.outer_ring_vel_z_D
                drivermode.en = 1
        elif abot_driver_way == 2:
            if side == 2:	#寻右线行驶
                drivermode.mask_x_left = self.Inner_ring_mask_x_left
                drivermode.mask_x_right = self.Inner_ring_mask_x_right
                drivermode.mask_y_top = self.Inner_ring_mask_y_top
                drivermode.mask_y_bot = self.Inner_ring_mask_y_bot
                drivermode.center_target = self.Inner_ring_center_target
                drivermode.vel_x = self.Inner_ring_vel_x
                drivermode.vel_y_P = self.Inner_ring_vel_y_P
                drivermode.vel_y_D = self.Inner_ring_vel_y_D
                drivermode.vel_z_P = self.Inner_ring_vel_z_P
                drivermode.vel_z_D = self.Inner_ring_vel_z_D
                drivermode.en = 4
        return drivermode
            

    def reconfigCB(self, config, level):
        self.outer_ring_center_target = config.outer_ring_center_target
        self.outer_ring_vel_z_P = config.outer_ring_vel_z_P
        self.outer_ring_vel_z_D = config.outer_ring_vel_z_D

        self.Inner_ring_center_target = config.Inner_ring_center_target
        self.Inner_ring_vel_y_P = config.Inner_ring_vel_y_P
        self.Inner_ring_vel_y_D = config.Inner_ring_vel_y_D
        self.Inner_ring_vel_z_P = config.Inner_ring_vel_z_P
        self.Inner_ring_vel_z_D = config.Inner_ring_vel_z_D 

        if self.last_ring != 0:
            drivermode_pub = self.ctrl_data(self.last_ring, self.last_side)
            self.driver_mode_pub.publish(drivermode_pub)

        return config


if __name__ == '__main__':
    try:
    	driver = ControlDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
    	pass
