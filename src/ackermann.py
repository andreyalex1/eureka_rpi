#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray
import numpy as np
from math import atan, sqrt, pi
import rospy
import threading
import time

class ackermann:
    def __init__(self):
        self.pub = rospy.Publisher('can_tx', UInt8MultiArray, queue_size=10)
        self.sub = rospy.Subscriber("cmd_vel", Twist, self.callback)
        self.vel_wheel = [0] * 6
        self.vel_wheel_filt = [0] * 6
        self.ang_wheel = [0] * 6
        self.ang_wheel_filt = [0] * 6
        self.l = [.41, 0, .385, .41, 0, .385]
        self.d = [.728, .780, .728, -.728, -.780, -.728]
        self.filt_thread = threading.Thread(target = self.filter, args = (), daemon=True)
        self.filt_thread.start()
        rospy.loginfo("Ackermann Started!")
    def __del__(self):
        rospy.loginfo("Ackermann Killed!")

    def send(self):
        for c in range(6):
            msg = UInt8MultiArray()
            arr = np.array([self.vel_wheel_filt[c], self.ang_wheel[c], 0.25], dtype = np.float16)
            data = bytes([c + 11]) + arr.tobytes()
            msg.data = data
            self.pub.publish(msg)
    def callback(self,data):
        vel_lin = data.linear.x
        vel_ang = data.angular.z
        # go straingt
        
        if(vel_ang == 0):
            self.vel_wheel = [vel_lin] * 6
            self.ang_wheel = [0] * 6
            self.vel_wheel[3] *= -1
            self.vel_wheel[4] *= -1
            self.vel_wheel[5] *= -1
        #turn
        else:
            rad = .6/vel_ang
            #turn center outside
            if(abs(rad) > .58):
                for c in range(6):
                    self.vel_wheel[c] = vel_lin / abs(rad) * sqrt(self.l[c]**2 + (rad + self.d[c]/2)**2)
                    self.ang_wheel[c] = atan(self.l[c]/ (rad + self.d[c] / 2)) * 180 / pi
                    if(c > 2):
                        self.vel_wheel[c] *= -1
            #turn center inside
            else:
                for c in range(6):
                    self.vel_wheel[c] = vel_lin * sqrt(self.l[c]**2 + (rad + self.d[c]/2)**2)
                    self.ang_wheel[c] = atan(self.l[c]/ (rad + self.d[c] / 2)) * 180 / pi
        self.ang_wheel[0] *= -1
        self.ang_wheel[3] *= -1
        self.ang_wheel[0] -= 2
        self.ang_wheel[2] -= 0
        self.ang_wheel[3] += 2
        self.ang_wheel[5] -= 0
        self.send()

    def filter(self):
        step = 0.05
        rate = rospy.Rate(20) # 10hz
        while not rospy.is_shutdown():
            for c in range (6):
                self.vel_wheel_filt[c] += step * (self.vel_wheel[c] - self.vel_wheel_filt[c])
            self.send()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('ackermann')
    obj = ackermann()
    rate = rospy.Rate(500) # 10hz
    rospy.spin()