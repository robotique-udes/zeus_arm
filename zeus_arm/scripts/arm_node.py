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
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class ArmNode():

	def __init__(self):

		rospy.loginfo("Initialized node")
		self.robot = RoboticArm()
		self.pub_cmd = rospy.Publisher('/motor_cmd',Float32MultiArray, queue_size=10)

		rospy.Subscriber("/teleop_cmd",Twist, self.set_joint_cmd)

		# Control loop @100Hz
		rospy.Timer(rospy.Duration(1.0/10),self.speed_controller)
		# Jacobian matrix @500Hz
		rospy.Timer(rospy.Duration(1.0/20),self.jacobian_matrix)

	def speed_controller(self,event):

		cmd = self.robot.speed_controller()
		rospy.loginfo("Command to publish : ")
		rospy.loginfo(cmd)
		self.pub_cmd.publish(cmd)


	def jacobian_matrix(self,event):
		self.robot.jacobian_matrix()

	def set_joint_cmd(self,msg):

		# Validate info received (comment out)
		rospy.loginfo("Data received : ")
		rospy.loginfo(msg)

		# Create command structure
		cmd = np.zeros((6,1),dtype=np.float32)
		cmd[0] = msg.linear.x
		cmd[1] = msg.linear.y
		cmd[2] = msg.linear.z

		# Set received command
		self.robot.ref_cmd = cmd


if __name__ == '__main__':
	try:
		rospy.init_node('arm_node',anonymous=True)
		node = ArmNode()
		rospy.spin()

	except rospy.ROSInterruptionException:
		pass


