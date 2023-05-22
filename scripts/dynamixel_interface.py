#!/usr/bin/env python

import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class dynamixelInterface:
    def __init__(self):
        self.a = 36.0/180.0*math.pi
        #self.position_list = [60.0/180.0*math.pi]
        #self.cnt = 0
        self.count = 0
        self.downcount = 10
        self.i = 0
        self.trajectory_pub = rospy.Publisher("dynamixel_workbench/joint_trajectory", JointTrajectory, queue_size=1)
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.points = [None]
        self.trajectory_point = JointTrajectoryPoint()
        self.initial_position = [0] * 4
        self.initialized_flag = False
        self.joint_state_sub = rospy.Subscriber("dynamixel_workbench/joint_states", JointState, self.jointStateCallback)        
        self.timer = rospy.Timer(rospy.Duration(2.0), self.timerCallback)

    def timerCallback(self, event):
        # self.cnt = (self.cnt + 1) % len(self.position_list)
        self.count = self.count + 1
        self.trajectory_msg.joint_names = ['servo0', 'servo1','servo2', 'servo3']
        #self.trajectory_point.positions = [1.0,-1.0]
        if self.count <= 10:
            self.trajectory_point.positions = [self.a *self.count + self.initial_position[0], -self.a*self.count + self.initial_position[1], self.a*self.count + self.initial_position[2], -self.a*self.count + self.initial_position[3]]
        elif self.count >= 10 and self.count <= 20:
            self.downcount = self.downcount - 1
            self.trajectory_point.positions = [self.a *self.downcount + self.initial_position[0], -self.a*self.downcount + self.initial_position[1], self.a*self.downcount + self.initial_position[2], -self.a*self.downcount + self.initial_position[3]]
        else:
            self.trajectory_point.positions = [self.initial_position[0],self.initial_position[1],self.initial_position[2],self.initial_position[3]]

        #self.trajectory_point.positions = [self.position_list[self.cnt], self.position_list[self.cnt], self.position_list[self.cnt], self.position_list[self.cnt]]
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
            self.initialized_flag = True

if __name__ == '__main__':
    rospy.init_node("dynamixel_interface_node")
    node = dynamixelInterface()
    rospy.spin()

