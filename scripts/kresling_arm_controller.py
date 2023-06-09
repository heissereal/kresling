#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import copy

class dynamixelInterface:
    def __init__(self):
        self.pose = PoseStamped()
        self.mocap_sub = rospy.Subscriber("mocap/pose", PoseStamped, self.mocapCallback)
        self.joy = Joy()
        self.joy_state_vec = Vector3()
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback)
        self.euler_vec = Vector3()

        #self.degree = 20.0/180.0*math.pi
        self.cnt_deg = 0.0
        self.prev_degree = [0] * 4
        self.degree = [0] * 4
        self.control_degree = [0] * 4
        self.output = Vector3()

        self.control_mode = rospy.get_param("~control_mode", True)
        
        self.height = 0.0
        self.min_height = rospy.get_param("~min_height", 0.165)
        self.max_height = rospy.get_param("~max_height", 0.44)
        self.joy_button_flag = 0
        
        self.mode = False
        self.joy = None

        self.roll_target = 0.0
        
        self.max_roll = rospy.get_param("~max_roll", 0.45)
        self.pitch_target = 0.0
        self.max_pitch = rospy.get_param("~max_pitch", 0.45)
        self.Kp = rospy.get_param("~rot_p_gain", 0.2)

        if not self.control_mode:
            self.Kp = 0
            self.min_height = -1
            self.max_height = 100

        self.i = 0
        self.trajectory_pub = rospy.Publisher("dynamixel_workbench/joint_trajectory", JointTrajectory, queue_size=1)
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.points = [None]
        self.trajectory_point = JointTrajectoryPoint()

        self.initial_position = [0] * 4
        self.initialized_flag = False
        self.joint_state_sub = rospy.Subscriber("dynamixel_workbench/joint_states", JointState, self.jointStateCallback)        

        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)



    def mocapCallback(self, msg):
        self.pose = msg

    def joyCallback(self, msg):
        self.joy = msg
        
    def joy_state(self):
        joy_button_start = self.joy.buttons[2]
        joy_button_stop = self.joy.buttons[1]
        joy_button_down = self.joy.buttons[0]
        if joy_button_start == 1:
            self.joy_button_flag = 1
        if joy_button_stop == 1:
            self.joy_button_flag = 0
        if joy_button_down == 1:
            self.joy_button_flag = 2
            
        self.joy_state_vec.x = -self.joy.axes[0]  # roll
        self.joy_state_vec.y = self.joy.axes[1] # pitch
        # print("joy_state {}".format(self.joy_button_flag))

    def height_state(self):
        self.height = self.pose.pose.position.z
        # print("height :{}".format(self.height))
        
        
    def quaternion_to_euler(self):
        quaternion_x = self.pose.pose.orientation.x
        quaternion_y = self.pose.pose.orientation.y
        quaternion_z = self.pose.pose.orientation.z
        quaternion_w = self.pose.pose.orientation.w
        euler = tf.transformations.euler_from_quaternion((quaternion_x,quaternion_y,quaternion_z,quaternion_w))
        self.euler_vec.x = euler[0]
        self.euler_vec.y = euler[1]
        self.euler_vec.z = euler[2]
        #print("euler: {}, {}, {}".format(self.euler_vec.x, self.euler_vec.y, self.euler_vec.z))


    def P_control(self):
        self.output.x = - (self.euler_vec.x - self.roll_target) * self.Kp
        self.output.y = - (self.euler_vec.y - self.pitch_target)* self.Kp

    def switch_mode(self):
        if not self.mode:
            self.roll_target = 0.0
            self.pitch_target = 0.0
        else:
            self.roll_target = self.max_roll * self.joy_state_vec.x
            self.pitch_target = self.max_pitch * self.joy_state_vec.y

    def up_and_down_mode(self):

        #print("self.joy_button_flag: {}".format(self.joy_button_flag))
        if self.joy_button_flag == 1:
            self.mode = False
            self.cnt_deg = self.cnt_deg + (2.0/180.0*math.pi)
        elif self.joy_button_flag == 2:
            self.mode = False
            self.cnt_deg = self.cnt_deg - (2.0/180.0*math.pi)

    def timerCallback(self, event):

        if self.joy is None:
            return

        self.joy_state()
        self.height_state()
        self.quaternion_to_euler()
        self.switch_mode()
        self.P_control()

        self.trajectory_msg.joint_names = ['servo0','servo1','servo2','servo3']

        if self.initialized_flag:
            if self.height >= self.min_height and self.height <= self.max_height:
                if self.joy_button_flag != 0:

                    self.up_and_down_mode()
                    self.prev_degree = copy.copy(self.degree)
                    self.degree[0]  =   self.cnt_deg - rate * self.output.y + self.initial_position[0]
                    self.degree[1]  =  -self.cnt_deg + rate * self.output.x + self.initial_position[1]
                    self.degree[2]  =   self.cnt_deg + rate * self.output.y + self.initial_position[2]
                    self.degree[3]  =  -self.cnt_deg - rate * self.output.x + self.initial_position[3]
                    self.trajectory_point.positions = [self.degree[0],self.degree[1], self.degree[2], self.degree[3]]
                else:
                    if self.joy.buttons[6] == 1:
                        #print("---------control_mode-------------")
                        self.mode = True
                        self.control_degree[0] = self.control_degree[0] - self.output.y
                        self.control_degree[1] = self.control_degree[1] + self.output.x
                        self.control_degree[2] = self.control_degree[2] + self.output.y
                        self.control_degree[3] = self.control_degree[3] - self.output.x

                        #print("control degree: {}; prev degree: {}".format(self.control_degree, self.prev_degree))

                        self.trajectory_point.positions = [self.control_degree[0],self.control_degree[1], self.control_degree[2], self.control_degree[3]]

                    else:
                        self.mode = False
                        self.control_degree = copy.copy(self.prev_degree)
                        self.trajectory_point.positions = [self.prev_degree[0],self.prev_degree[1], self.prev_degree[2], self.prev_degree[3]]
            else:
                self.trajectory_point.positions = [self.prev_degree[0],self.prev_degree[1], self.prev_degree[2], self.prev_degree[3]]
            #print("degree: {}, {}, {}, {}".format(self.trajectory_point.positions[0], self.trajectory_point.positions[1], self.trajectory_point.positions[2], self.trajectory_point.positions[3]))
        
        self.trajectory_point.velocities = [0.0,0.0,0.0,0.0]
        self.trajectory_point.accelerations = [0.0, 0.0,0.0,0.0]
        self.trajectory_point.effort = [0.0, 0.0,0.0,0.0]
        self.trajectory_point.time_from_start = rospy.Time(0.1)
            
        self.trajectory_msg.points[0] = self.trajectory_point
        self.trajectory_pub.publish(self.trajectory_msg)

    def jointStateCallback(self, msg):
        if not self.initialized_flag:
            for i in range(len(self.initial_position)):
                self.initial_position[i] = msg.position[i]
            self.prev_degree = self.initial_position
            self.initialized_flag = True

if __name__ == '__main__':
    rospy.init_node("dynamixel_interface_node")
    node = dynamixelInterface()
    rospy.spin()

