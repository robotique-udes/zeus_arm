#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Sat November 14 16:07:35 2020
# @author: santi


"""
@package robot_arm

------------------------------------

Package containing the rover's arm class

"""

import rospy
import numpy as np
from arm_class import RoboticArm
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist
from zeus_arm.msg import Floats
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class ArmNode():

	def __init__(self):

		rospy.loginfo("Initialized node")
		self.robot = RoboticArm()
		self.pub_cmd = rospy.Publisher('/motor_cmd', Float32MultiArray, queue_size=10)
		rospy.Subscriber("/teleop_cmd", Twist, self.set_joint_cmd)
		rospy.Subscriber("/zeus_arm/joint_states", JointState ,self.update_joint_states)

		# Control loop @40Hz
		rospy.Timer(rospy.Duration(1.0/40),self.speed_controller)

	def speed_controller(self,event):
		cmd = Float32MultiArray()
		cmd.data = self.robot.speed_controller()
		self.pub_cmd.publish(cmd)


	def jacobian_matrix(self):
		self.robot.jacobian_matrix()

	def set_joint_cmd(self,msg):

		# Create command structure
		cmd = np.zeros((6,1),dtype=np.float32)
		cmd[0] = msg.linear.x
		cmd[1] = msg.linear.y
		cmd[2] = msg.linear.z
		cmd[3] = msg.angular.x
		cmd[4] = msg.angular.y
		cmd[5] = msg.angular.z 

		# Set received command
		self.robot.ref_cmd = cmd

	def update_joint_states(self, msg):

		self.joint_angles = list(msg.position)
		self.jacobian_matrix()

if __name__ == '__main__':
	try:
		rospy.init_node('arm_node',anonymous=True)
		node = ArmNode()
		rospy.spin()

	except rospy.ROSInterruptionException:
		pass


