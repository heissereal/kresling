#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

class rosserialInterface:
    def __init__(self):
        self.servo_speeds = [90, 90, 90, 90]
        self.pose = PoseStamped()
        self.joy = Joy()
        self.mocap_sub = rospy.Subscriber("mocap/pose", PoseStamped, self.mocapCallback)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback)

        self.servo0_msg = Int16()
        self.servo1_msg = Int16()
        self.servo2_msg = Int16()
        self.servo3_msg = Int16()
        self.servo0_pub = rospy.Publisher("servo_controller/servo0/command", Int16, queue_size=1)
        self.servo1_pub = rospy.Publisher("servo_controller/servo1/command", Int16, queue_size=1)
        self.servo2_pub = rospy.Publisher("servo_controller/servo2/command", Int16, queue_size=1)
        self.servo3_pub = rospy.Publisher("servo_controller/servo3/command", Int16, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

    def mocapCallback(self, msg):
        self.pose = msg

    def joyCallback(self, msg):
        self.joy = joy

    def timerCallback(self, event):
        self.servo0_msg.data = 90
        self.servo1_msg.data = 90
        self.servo2_msg.data = 90
        self.servo3_msg.data = 90
        self.servo0_pub.publish(self.servo0_msg)
        self.servo1_pub.publish(self.servo1_msg)
        self.servo2_pub.publish(self.servo2_msg)
        self.servo3_pub.publish(self.servo3_msg)


if __name__ == '__main__':
    rospy.init_node("rosserial_interface_node")
    node = rosserialInterface()
    rospy.spin()

