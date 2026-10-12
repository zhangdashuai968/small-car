#!/usr/bin/env python

import rospy
import json
import threading
import sys
import serial
#import struct

class xArmCtrl(object):
    def __init__(self, port='/dev/ttyACM0', baud=115200):
        self.__ser = serial.Serial(port, baud, timeout=0.5)
        self.__ser.bytesize = 8
        self.__ser.parity = serial.PARITY_NONE
        self.__ser.stopbits = 1
        if self.__ser.isOpen():
            self.__ser.close()
        try:
            self.__ser.open()
        except serial.SerialException, msg:
            print 'Serial open failed! :' + msg
            sys.exit()

        #Add rx data threading 
        rx_th = threading.Thread(target = self.SerialRecvieCallback)
        rx_th.setDaemon(True)
        rx_th.start()
       
        #test data
        #r = rospy.Rate(1)
        #cnt = 0
        #data = {"RikibotxArmCmd": 1, "Pose": {"x": 100,"y": 100,"z": 100, "rotary": 90},"time": 1000}
        #data ="test error"
        #print(json.dumps(data))
        #while not rospy.is_shutdown():
        #    rospy.loginfo("send data: %d", cnt)
        #    self.__ser.write(json.dumps(data))
            #self.__ser.write(data)
        #    cnt = cnt + 1
        #    r.sleep()

    def SerialWriteData(self, data):
        self.__ser.write(data)

    def SerialRecvieCallback(self):
        while True:
            #rx_data = self.__ser.readline()
            rx_len = self.__ser.inWaiting() 
            #self.__ser.flushInput()
            #if len(rx_data):
            if rx_len > 0:
                rx_data = self.__ser.read(rx_len)
                rx_data = rx_data.decode()
                try:
                    json_object = json.loads(rx_data)
                    rospy.loginfo("json data: %s", rx_data)
                except ValueError, e:
                    rospy.loginfo("rx data: %s", rx_data)

if __name__ == '__main__':
    rospy.init_node("Rikibot_xArm_Controller")
    try:
        xArm = xArmCtrl()
    except Exception.message:
        print "...........xxx............", Exception.message
        pass
