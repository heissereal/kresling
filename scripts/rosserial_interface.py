#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
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
        self.euler_vector = Vector3()

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

    def mocapCallback(self, msg):
        self.pose = msg

    def joyCallback(self, msg):
        self.joy = msg

    def quaternion_to_euler(self):
        quaternion_x = self.pose.pose.orientation.x
        quaternion_y = self.pose.pose.orientation.y
        quaternion_z = self.pose.pose.orientation.z
        quaternion_w = self.pose.pose.orientation.w
        e = tf.transformations.euler_from_quaternion((quaternion_x,quaternion_y,quaternion_z,quaternion_w))
        self.euler_vector.x = e[0]
        self.euler_vector.y = e[1]
        self.euler_vector.z = e[2]

    def P_control(self):
        Kp = 50
        output_x = - self.euler_vector.x*Kp
        output_y = - self.euler_vector.y*Kp
        #output_z = euler_vector.z*0.01
        self.servo0_msg.data = int(-output_y)
        self.servo2_msg.data = int(output_y)
        #if euler degree y > 0, output of servo0 is positive and output of servo2 is negative.
        #servo0 and 2 rotate and expand when servo_degree > 90 

        self.servo1_msg.data = int(output_x)
        self.servo3_msg.data = int(-output_x)
        #if euler degree x > 0, output of servo1 is positive and output of servo3 is negative.
        #servo1 and 3 rotate and expand when servo_degree < 90 
        
            
    def timerCallback(self, event):
        
        self.quaternion_to_euler()
        self.P_control()
        self.servo0_pub.publish(self.servo0_msg)
        self.servo1_pub.publish(self.servo1_msg)
        self.servo2_pub.publish(self.servo2_msg)
        self.servo3_pub.publish(self.servo3_msg)


if __name__ == '__main__':
    rospy.init_node("rosserial_interface_node")
    node = rosserialInterface()
    rospy.spin()

