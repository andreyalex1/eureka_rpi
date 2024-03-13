#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray
import numpy as np
from math import atan, sqrt, pi
import rospy

class ackermann:
    def __init__(self):
        self.pub = rospy.Publisher('can_tx', UInt8MultiArray, queue_size=10)
        self.sub = rospy.Subscriber("cmd_vel", Twist, self.callback)
        self.vel_wheel = [0] * 6
        self.ang_wheel = [0] * 6
        self.l = [.41, 0, .385, .41, 0, .385]
        self.d = [.728, .780, .728, -.728, -.780, -.728]
        rospy.loginfo("Ackermann Started!")
    def __del__(self):
        rospy.loginfo("Ackermann Killed!")

    def send(self):
        for c in range(6):
            msg = UInt8MultiArray()
            arr = np.array([self.vel_wheel[c], self.ang_wheel[c]], dtype = np.float16)
       #     print(arr)
            data = bytes([c + 11]) + arr.tobytes()
            msg.data = data
      #      print(msg)
            self.pub.publish(msg)
    def callback(self,data):
    #    print(data)
        vel_lin = data.linear.x
        vel_ang = data.angular.z
  #      print(vel_lin, vel_ang)
        # go straingt
        
        if(vel_ang == 0):
            self.vel_wheel = [vel_lin] * 6
            self.ang_wheel = [0] * 6
        #turn
        else:
            rad = .6/vel_ang
            #turn center outside
            if(abs(rad) > .58):
                for c in range(6):
                    self.vel_wheel[c] = vel_lin / rad * sqrt(self.l[c]**2 + (rad + self.d[c]/2)**2)
                    self.ang_wheel[c] = atan(self.l[c]/ (rad + self.d[c] / 2)) * 180 / pi
            else:
                for c in range(6):
                    self.vel_wheel[c] = vel_lin / rad * sqrt(self.l[c]**2 + (rad + self.d[c]/2)**2)
                    self.ang_wheel[c] = atan(self.l[c]/ (rad + self.d[c] / 2)) * 180 / pi
        self.send()
        print(self.vel_wheel)
        print(self.ang_wheel)
        print("-------------------------")

if __name__ == "__main__":
    rospy.init_node('ackermann')
    obj = ackermann()
    rate = rospy.Rate(500) # 10hz
    rospy.spin()