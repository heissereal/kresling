#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class dynamixelInterface:
    def __init__(self):
        self.position_list = [0.0, 1.0, 0.0, -1.0]

        self.cnt = 0
        self.trajectory_pub = rospy.Publisher("dynamixel_workbench/joint_trajectory", JointTrajectory, queue_size=1)
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.points = [None]
        self.trajectory_point = JointTrajectoryPoint()

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

    def timerCallback(self, event):
        self.cnt = (self.cnt + 1) % len(self.position_list)

        self.trajectory_msg.joint_names = ['servo0', 'servo1', 'servo2', 'servo3']

        self.trajectory_point.positions = [self.position_list[self.cnt], self.position_list[self.cnt], self.position_list[self.cnt], self.position_list[self.cnt]]
        self.trajectory_point.velocities = [0.0, 0.0, 0.0, 0.0]
        self.trajectory_point.accelerations = [0.0, 0.0, 0.0, 0.0]
        self.trajectory_point.effort = [0.0, 0.0, 0.0, 0.0]
        self.trajectory_point.time_from_start = rospy.Time(0.1)

        self.trajectory_msg.points[0] = self.trajectory_point
        self.trajectory_pub.publish(self.trajectory_msg)

if __name__ == '__main__':
    rospy.init_node("dynamixel_interface_node")
    node = dynamixelInterface()
    rospy.spin()

