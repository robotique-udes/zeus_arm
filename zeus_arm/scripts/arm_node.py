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
from zeus_arm.msg import Command

class ArmNode():

    def __init__(self):
        '''
        Node class for robot arm instance
        '''

        rospy.loginfo("Initialized node")
        self.robot = RoboticArm()
        self.cmd = np.zeros((6,1),dtype=np.float64)
        self.ref_cmd = np.zeros((6,1),dtype=np.float64)
        self.ctrl_mode = 2

        # Init subscripers
        rospy.Subscriber("/zeus_arm/cmd_vel", Command, self.set_cmd)
        rospy.Subscriber("/zeus_arm/joint_states", JointState ,self.update_joint_states)

        # Init publishers
        self.j1_pub = rospy.Publisher('/zeus_arm/joint_1_velocity_controller/command', Float64, queue_size=10)
        self.j2_pub = rospy.Publisher('/zeus_arm/joint_2_velocity_controller/command', Float64, queue_size=10)
        self.j3_pub = rospy.Publisher('/zeus_arm/joint_3_velocity_controller/command', Float64, queue_size=10)
        self.j4_pub = rospy.Publisher('/zeus_arm/joint_4_velocity_controller/command', Float64, queue_size=10)
        self.j5_pub = rospy.Publisher('/zeus_arm/joint_5_velocity_controller/command', Float64, queue_size=10)
        
        # Control loop @40Hz
        rospy.Timer(rospy.Duration(0.3),self.speed_controller)


    def speed_controller(self,event):
        """
        Velocity control loop
        """
        # TODO : Move Arduino position publishing inside de command callback  
        if self.ctrl_mode == 2:
            self.robot.ref_cmd = self.ref_cmd
            self.cmd = self.robot.speed_controller()

        self.send_cmd(self.cmd)


    def send_cmd(self, cmd):
        '''
        Publishes commands
        '''
        self.j1_pub.publish(cmd[0])
        self.j2_pub.publish(cmd[1])
        self.j3_pub.publish(cmd[2])
        self.j4_pub.publish(cmd[3])
        self.j5_pub.publish(cmd[4])

    def set_cmd(self,msg):
        '''
        Callback from joystick
        ----------
        Parameters
        ----------
        msg: Command
             Command structure containing control mode and command
        '''

        if msg.mode == 1:
            self.ctrl_mode = 1
            self.cmd[0] = msg.cmd.linear.x
            self.cmd[1] = msg.cmd.linear.y
            self.cmd[2] = msg.cmd.linear.z
            self.cmd[3] = msg.cmd.angular.x
            self.cmd[4] = msg.cmd.angular.y

        else:
            self.ctrl_mode = 2
            self.ref_cmd[0] = msg.cmd.linear.x
            self.ref_cmd[1] = msg.cmd.linear.y
            self.ref_cmd[2] = msg.cmd.linear.z
            self.ref_cmd[3] = msg.cmd.angular.x
            self.ref_cmd[4] = msg.cmd.angular.y
            self.ref_cmd[5] = msg.cmd.angular.z 



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
        # self.robot.joint_angles = np.array(list(msg.position))
        self.robot.joint_angles[0] = msg.position[0]
        self.robot.joint_angles[1] = msg.position[1]
        self.robot.joint_angles[2] = msg.position[2]
        self.robot.joint_angles[3] = msg.position[3]
        self.robot.joint_angles[4] = msg.position[4]
        
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


