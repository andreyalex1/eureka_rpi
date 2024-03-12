#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

import os
import can
import rospy
from std_msgs.msg import UInt8MultiArray
import numpy as np

class can_transceiver:
    def __init__(self):
        self.pub = rospy.Publisher('can_rx', UInt8MultiArray, queue_size=10)
        self.sub = rospy.Subscriber("can_tx", UInt8MultiArray, self.callback)
        os.system('sudo ip link set can0 up type can bitrate 1000000 restart-ms 1000')
        self.can = can.interface.Bus(channel = 'can0', bustype = 'socketcan')  # socketcan_native
        rospy.loginfo("CAN Started!")
    def __del__(self):
        os.system('sudo ifconfig can0 down')
        rospy.loginfo("CAN Killed!")
    def recv(self, timeout):
        return can.recv(timeout)
    def callback(self, msg):
        send = self.can.Message(msg.data[0],data = msg.data[1:9] , extended_id=False)
        self.can.send(send)
    def spin(self):
        rate = rospy.Rate(500) # 10hz
        while not rospy.is_shutdown():
            arr = UInt8MultiArray()
            msg = self.can.recv(10.0)
            if(msg is  None):
                rate.sleep()
            else:
                arr.data = bytes([msg.arbitration_id])    
                arr.data += msg.data
                self.pub.publish(arr)   


if __name__ == "__main__":
    rospy.init_node('can_transceiver')
    obj = can_transceiver()
    obj.spin()