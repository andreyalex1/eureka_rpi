#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import threading
import rospy
import math

class arm_decoder:
    def __init__(self):
        self.velocities = [0] * 6
        self.efforts = [0] * 6
        self.positions = [0] * 6
        self.pub = rospy.Publisher('dc_states', JointState, queue_size=10)
        self.sub = self.sub = rospy.Subscriber("can_rx",  UInt8MultiArray, self.callback)
        self.pub_thread = threading.Thread(target = self.publisher, args = (), daemon=True)
        self.pub_thread.start()
        rospy.loginfo("DC_Decoder Started!")
    def __del__(self):
        rospy.loginfo("DC_Decoder Killed!")
    def callback(self, arr):
        index = int(arr.data[0])
        if( 10 < index < 17):
            temp = np.frombuffer(arr.data[5:7], dtype=np.float16)[0]
            if(math.isnan(temp)):
                temp = 0
            self.positions[index - 11] = temp
            self.velocities[index - 11] = np.frombuffer(arr.data[1:3], dtype=np.float16)[0]
            self.efforts[index - 11] = np.frombuffer(arr.data[3:5], dtype=np.float16)[0]
    def publisher(self):
        rate = rospy.Rate(20) # 10hz
        while not rospy.is_shutdown():
            message = JointState()
            message.name = ['DC1','DC2','DC3','DC4','DC5','DC6']
            message.velocity = self.velocities
            message.effort = self.efforts
            message.position = self.positions
            self.pub.publish(message)
            rate.sleep()
    
if __name__ == "__main__":
    rospy.init_node('dc_decoder')
    obj = arm_decoder()
    rate = rospy.Rate(500) # 10hz
    rospy.spin()