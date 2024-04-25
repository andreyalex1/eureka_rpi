#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import threading
import rospy

class arm_decoder:
    def __init__(self):
        self.angles = [0] * 6
        self.pub = rospy.Publisher('arm_joint_states', JointState, queue_size=10)
        self.sub = self.sub = rospy.Subscriber("can_rx",  UInt8MultiArray, self.callback)
        self.pub_thread = threading.Thread(target = self.publisher, args = (), daemon=True)
        self.pub_thread.start()
        rospy.loginfo("Arm_Decoder Started!")
    def __del__(self):
        rospy.loginfo("Arm_Decoder Killed!")
    def callback(self, arr):
        index = int(arr.data[0])
        if( 20 < index < 27):
            self.angles[index - 21] = np.frombuffer(arr.data[1:3], dtype=np.float16)[0]
    def publisher(self):
        rate = rospy.Rate(20) # 10hz
        while not rospy.is_shutdown():
            message = JointState()
            message.name = ['Revolute1','Revolute2','Revolute3','Revolute4','Revolute5','Slider1']
            message.position = self.angles
            self.pub.publish(message)
            rate.sleep()
    
if __name__ == "__main__":
    rospy.init_node('arm_decoder')
    obj = arm_decoder()
    rate = rospy.Rate(500) # 10hz
    rospy.spin()