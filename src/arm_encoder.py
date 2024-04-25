#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import threading
import rospy

#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import threading
import rospy

class arm_encoder:
    def __init__(self):
        self.angles = [0] * 6
        self.velocities = [0] * 6
        self.update_flag = 0
        self.sub = rospy.Subscriber('arm_joint_commands', JointState, self.callback)
        self.pub = self.sub = rospy.Publisher("can_tx",  UInt8MultiArray, queue_size=10)
        self.pub_thread = threading.Thread(target = self.publisher, args = (), daemon=True)
        self.pub_thread.start()
        rospy.loginfo("Arm_Encoder Started!")
    def __del__(self):
        rospy.loginfo("Arm_Encoder Killed!")
    def callback(self, msg):
        self.velocities = msg.velocity
        self.update_flag = 0
    def publisher(self):
        rate = rospy.Rate(20) # 10hz
        while not rospy.is_shutdown():
            if self.update_flag > 1:
                self.velocities = [0] * 6
            for c in range(6):
                message = UInt8MultiArray()
                arr = np.array([self.velocities[c],], dtype = np.float16)
                data = bytes([c + 21]) + arr.tobytes()
                message.data = data
                self.pub.publish(message)
                rate.sleep()
            self.update_flag +=1
    
if __name__ == "__main__":
    rospy.init_node('arm_encoder')
    obj = arm_encoder()
    rate = rospy.Rate(500) # 10hz
    rospy.spin()