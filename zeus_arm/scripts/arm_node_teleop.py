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
from std_msgs.msg import Float64MultiArray
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
        self.ctrl_mode = 1
        self.encoders_offset = [0.20, 0.60, -0.178, -2.54]

        # Init subscripers
        rospy.Subscriber("/zeus_arm/cmd_vel", Command, self.set_cmd)
        rospy.Subscriber("/zeus_arm/joint_positions", Float64MultiArray ,self.update_joint_states)

        # Init publishers
        self.cmd_pub = rospy.Publisher('/zeus_arm/joint_commands', Float64MultiArray, queue_size=10)
        
        # Control loop @40Hz
        rospy.Timer(rospy.Duration(0.3),self.speed_controller)


    def speed_controller(self,event):
        """
        Velocity control loop
        """
        # TODO : Move Arduino position publishing inside de command callback  
        if self.ctrl_mode == 2:
            self.robot.ref_cmd = self.ref_cmd
            cmd = self.robot.speed_controller()
            self.cmd[0] = cmd[0]
            self.cmd[1] = cmd[1]
            self.cmd[2] = cmd[2]
            self.cmd[3] = cmd[3]
            self.cmd[4] = cmd[4]

        self.send_cmd(self.cmd)


    def send_cmd(self, cmd):
        '''
        Publishes commands
        '''
        if type(cmd) is np.ndarray:
            cmd = cmd.flatten().tolist()
        
        cmd_array = Float64MultiArray()
        cmd_array.data = cmd
        self.cmd_pub.publish(cmd_array)

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
            # Gripper
            self.cmd[5] = msg.gripper_cmd

        else:
            self.ctrl_mode = 2
            self.ref_cmd[0] = msg.cmd.linear.x
            self.ref_cmd[1] = msg.cmd.linear.y
            self.ref_cmd[2] = msg.cmd.linear.z
            self.ref_cmd[3] = msg.cmd.angular.x
            self.ref_cmd[4] = msg.cmd.angular.y
            self.ref_cmd[5] = msg.cmd.angular.z 
            self.cmd[5] = msg.gripper_cmd



    def update_joint_states(self, msg):
        '''
        Callback from encoders
        ----------
        Parameters
        ----------
        msg: Float64MultiArray
             States for all joints commind from simulation
        '''

        # Update joint angles
        self.robot.joint_angles[0] = msg.data[0] + self.encoders_offset[0] 
        self.robot.joint_angles[1] = msg.data[1] + self.encoders_offset[1]
        self.robot.joint_angles[2] = msg.data[2] + self.encoders_offset[2]
        self.robot.joint_angles[3] = msg.data[3] + self.encoders_offset[3]
        #self.robot.joint_angles[4] = msg.data[4]
        
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


