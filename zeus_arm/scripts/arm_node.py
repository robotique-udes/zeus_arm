#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Sat November 14 16:07:35 2020
# @author: Santiago Moya        santiago.moya@usherbrooke.ca


"""
@package zeus_arm

------------------------------------

ROS node containing robot arm instance 

"""

import rospy
import numpy as np
from arm_class import RoboticArm
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist
from zeus_arm.msg import Floats
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class ArmNode():

    def __init__(self):
        '''
        Node class for robot arm instance
        '''

        rospy.loginfo("Initialized node")
        self.robot = RoboticArm()

        # Init subscripers
        rospy.Subscriber("/zeus_arm/cmd_vel", Twist, self.set_joint_cmd)
        rospy.Subscriber("/zeus_arm/joint_states", JointState ,self.update_joint_states)

        # Init publishers
        self.j1_pub = rospy.Publisher('/zeus_arm/joint_1_position_controller/command', Float64, queue_size=10)
        self.j2_pub = rospy.Publisher('/zeus_arm/joint_2_position_controller/command', Float64, queue_size=10)
        self.j3_pub = rospy.Publisher('/zeus_arm/joint_3_position_controller/command', Float64, queue_size=10)
        self.j4_pub = rospy.Publisher('/zeus_arm/joint_4_position_controller/command', Float64, queue_size=10)
        self.j5_pub = rospy.Publisher('/zeus_arm/joint_5_position_controller/command', Float64, queue_size=10)
        
        # Control loop @40Hz
        rospy.Timer(rospy.Duration(1.0/50),self.speed_controller)


    def speed_controller(self,event):
        """
        Velocity control loop
        """
        # TODO : Move Arduino position publishing inside de command callback  
        cmd = self.robot.speed_controller()
        self.send_cmd(cmd)


    def send_cmd(self, cmd):
        '''
        Publishes commands
        '''
        self.j1_pub.publish(cmd[0])
        self.j2_pub.publish(cmd[1])
        self.j3_pub.publish(cmd[2])
        self.j4_pub.publish(cmd[3])
        self.j5_pub.publish(cmd[4])

    def set_joint_cmd(self,msg):
        '''
        Callback from joystick
        ----------
        Parameters
        ----------
        msg: Twist
             Cartesian command for end effector
        '''

        # Create command structure
        cmd = np.zeros((6,1),dtype=np.float64)
        cmd[0] = msg.linear.x
        cmd[1] = msg.linear.y
        cmd[2] = msg.linear.z
        cmd[3] = msg.angular.x
        cmd[4] = msg.angular.y
        cmd[5] = msg.angular.z 

        self.robot.ref_cmd = cmd

    def update_joint_states(self, msg):
        '''
        Callback from encoders
        ----------
        Parameters
        ----------
        msg: JointState
             States for all joints commind from simulation
        '''

        # Update joint angles
        self.robot.joint_angles = np.array(list(msg.position))
        
        # Update theta parameters
        self.robot.t_dh[0] = 0.
        self.robot.t_dh[1] = self.robot.joint_angles[0]
        self.robot.t_dh[2] = self.robot.joint_angles[1] + np.pi/2
        self.robot.t_dh[3] = self.robot.joint_angles[2]
        self.robot.t_dh[4] = self.robot.joint_angles[3] - np.pi/2
        self.robot.t_dh[5] = self.robot.joint_angles[4]

if __name__ == '__main__':
    try:
        rospy.init_node('arm_node',anonymous=True)
        node = ArmNode()
        rospy.spin()

    except rospy.ROSInterruptionException:
        pass


