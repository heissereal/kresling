#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray

class hand_Interface:
    def __init__(self):
        self.sensor_msg = [0]*4
        self.sensor_sub = rospy.Subscriber("sensor", Int16MultiArray, self.sensorCallback)

        self.servo0_msg = Int16()
        self.servo1_msg = Int16()
        self.servo2_msg = Int16()
        self.servo3_msg = Int16()
        self.servo0_pub = rospy.Publisher("servo0/command", Int16, queue_size=1)
        self.servo1_pub = rospy.Publisher("servo1/command", Int16, queue_size=1)
        self.servo2_pub = rospy.Publisher("servo2/command", Int16, queue_size=1)
        self.servo3_pub = rospy.Publisher("servo3/command", Int16, queue_size=1)

        self.servo_msg = [0]*4
        #self.servo_msg_array = Int16MultiArray()
        #self.servo_pub = rospy.Publisher("servo/command", Int16MultiArray, queue_size = 1)
        
       
    def sensorCallback(self,msg):
        self.sensor_msg = msg
        #print(self.sensor_msg.data[0])
        # for i in range(3):
        #     self.servo_msg[i] = self.sensor_msg.data[i]
        self.servo0_msg.data = self.sensor_msg.data[0]
        self.servo1_msg.data = self.sensor_msg.data[1]
        self.servo2_msg.data = 180 - self.sensor_msg.data[2]
        self.servo3_msg.data = 180 - self.sensor_msg.data[3]
        self.servo0_pub.publish(self.servo0_msg)
        self.servo1_pub.publish(self.servo1_msg)
        self.servo2_pub.publish(self.servo2_msg)
        self.servo3_pub.publish(self.servo3_msg)
        
    

if __name__ == '__main__':
    rospy.init_node("hand_interface_node")
    node = hand_Interface()
    rospy.spin()

            
