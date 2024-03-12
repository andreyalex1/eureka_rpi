#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from geometry_msgs.msg import Twist
import numpy as np
from math import atan

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
    def callback(self,data):
        vel_lin = data.linear[0]
        vel_ang = data.angular[0]
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
                    vel_wheel[c] = vel_lin / rad * sqrt(self.l[c]**2 + (rad + self.d[c]/2)**2)
                    ang_wheel[c] = atan(self.l[c]/ (rad + self.d[c] / 2))
            else:
                for c in range(6):
                    vel_wheel[c] = vel_lin / rad * sqrt(self.l[c]**2 + (rad + self.d[c]/2)**2)
                    ang_wheel[c] = atan(self.l[c]/ (rad + self.d[c] / 2))