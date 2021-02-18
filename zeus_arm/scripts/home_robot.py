#!/usr/bin/env python

import sys
import time
import rospy
import numpy as np 
from std_srvs.srv import Empty
from std_msgs.msg import Float64

class HomeRobot():


    def __init__(self):

    	# Init publishers
        self.j1_pub = rospy.Publisher('/zeus_arm/joint_1_position_controller/command', Float64, queue_size=10)
        self.j2_pub = rospy.Publisher('/zeus_arm/joint_2_position_controller/command', Float64, queue_size=10)
        self.j3_pub = rospy.Publisher('/zeus_arm/joint_3_position_controller/command', Float64, queue_size=10)
        self.j4_pub = rospy.Publisher('/zeus_arm/joint_4_position_controller/command', Float64, queue_size=10)
        self.j5_pub = rospy.Publisher('/zeus_arm/joint_5_position_controller/command', Float64, queue_size=10)

    	# Define default home position
    	self.home_position = np.array([0.0, 0.0, 1.57, 1.57, 0.0], dtype=np.float64)

    	# Unpause Gazebo
    	rospy.loginfo("Unpausing Gazebo...")
    	rospy.wait_for_service('/gazebo/unpause_physics')
    	unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        resp = unpause_gazebo()
    	rospy.loginfo("Unpaused Gazebo.")

    	# Send initial position to robot
        success = self.home()

        if success == True :
        	rospy.loginfo("Successfully homed the robot")
        else :
        	rospy.loginfo("There was an error while homing the arm")

    def home(self):

    	try:
			self.j1_pub.publish(self.home_position[0])
			self.j2_pub.publish(self.home_position[1])
			self.j3_pub.publish(self.home_position[2])
			self.j4_pub.publish(self.home_position[3])
			self.j5_pub.publish(self.home_position[4])

			return True

        except:
        	return False



if __name__ == '__main__':
    rospy.init_node('init_robot', anonymous=True)
    node = HomeRobot()

