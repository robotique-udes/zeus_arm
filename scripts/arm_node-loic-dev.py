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
from zeus_arm.msg import Command, Cmd_arm
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class ArmNode():

    def __init__(self, simulation=False):
        '''
        Node class for robot arm instance
        '''
        if simulation:
            rospy.logwarn("Initialized node in simulation...")
        else:
            rospy.loginfo("Initialized node")

        # Zero speed publisher #10Hz
        rospy.loginfo("Setting zero speed publisher")
        self.zero_speed_pub = rospy.Publisher('/zeus_arm/zero_twist', Twist, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/10),self.zero_speed)

        self.robot = RoboticArm()
        self.cmd = np.zeros((6,1),dtype=np.float64)
        self.last_cmd = np.zeros((6,1),dtype=np.float64)
        self.ref_cmd = np.zeros((6,1),dtype=np.float64)
        self.last_ref_cmd = np.zeros((6,1),dtype=np.float64)
        self.ctrl_mode = 2
        self.calibration_done = True;
        self.last_change = 0.
        self.last_cmd = np.zeros((6,1),dtype=np.float64)

        # Init subscripers
        rospy.Subscriber("/zeus_arm/cmd_vel_mux", Twist, self.set_cmd)
        rospy.Subscriber("/zeus_arm/ctrl_mode", Int16, self.set_ctrl_mode)
        rospy.Subscriber("/zeus_arm/joint_states", JointState ,self.update_joint_states)
        rospy.Subscriber("/zeus_arm/calibration_done", Bool ,self.update_calibration_status)


        # Init publishers
        self.j1_pub = rospy.Publisher('/zeus_arm/joint_1_velocity_controller/command', Float64, queue_size=10)
        self.j2_pub = rospy.Publisher('/zeus_arm/joint_2_velocity_controller/command', Float64, queue_size=10)
        self.j3_pub = rospy.Publisher('/zeus_arm/joint_3_velocity_controller/command', Float64, queue_size=10)
        self.j4_pub = rospy.Publisher('/zeus_arm/joint_4_velocity_controller/command', Float64, queue_size=10)

        # Control loop @40Hz
        rospy.Timer(rospy.Duration(1.0/50),self.speed_controller)


        # Initialize configurable params
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure("zeus_arm")

        # Add variables to ddr(name, description, default value, min, max, edit_method)        
        # Model Settings
        self.ddr.add_variable("lambda_gain", "float", 0.1, 0., 10.)
        


        # Start Server
        self.ddr.start(self.dynamic_reconfigure_callback)
        rospy.sleep(1)

    def zero_speed(self, event):
        self.zero_speed_pub.publish(Twist())


    def dynamic_reconfigure_callback(self, config, level):

        '''
        Updates parameters value when changed by the user.
        ----------
        Parameters
        ----------
        config: dict
            Keys are param names and values are param values
        level: Unused
        -------
        Returns
        -------
        config: dict
            Keys are param names and values are param values
        '''
        # Update variables
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            # Update DDR server
            self.__dict__[var_name] = config[var_name]
            # Update robot class
            self.robot.lambda_gain = config[var_name]
        return config


    def speed_controller(self,event):
        """
        Velocity control loop
        """ 
        if self.ctrl_mode == 2:
            self.robot.ref_cmd = self.ref_cmd
            self.cmd = self.robot.speed_controller()

        self.send_cmd(self.cmd)



    def send_cmd(self, cmd):
        '''
        Publishes commands
        '''

        if(self.calibration_done):
            self.j1_pub.publish(cmd[0])
            self.j2_pub.publish(cmd[1])
            self.j3_pub.publish(cmd[2])
            self.j4_pub.publish(cmd[3])


    def set_cmd(self,msg):
        '''
        Callback from joystick
        ----------
        Parameters
        ----------
        msg: Command
             Command structure containing control mode and command
        '''

        if self.ctrl_mode == 1:
            self.cmd[0] = msg.linear.x
            self.cmd[1] = msg.linear.y
            self.cmd[2] = msg.linear.z
            self.cmd[3] = msg.angular.x
            self.cmd[4] = msg.angular.y

        else:
            self.ref_cmd[0] = msg.linear.x
            self.ref_cmd[1] = msg.linear.y
            self.ref_cmd[2] = msg.linear.z
            self.ref_cmd[3] = msg.angular.x
            self.ref_cmd[4] = msg.angular.y
            self.ref_cmd[5] = msg.angular.z

    def set_ctrl_mode(self,msg):
        '''
        Callback for control mode
        ----------
        Parameters
        ----------
        msg: Control mode
             Int containing control mode 
        '''
        self.ctrl_mode = msg.data

    def update_calibration_status(self, msg):
        '''
        Callback from calibration topic
        ----------
        Parameters
        ----------
        msg: Bool
             State of calibration. True = done, false = not done.
        '''
        self.calibration_done = msg.data


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
        self.robot.joint_angles[1] = msg.position[1]
        self.robot.joint_angles[0] = msg.position[0]
        self.robot.joint_angles[2] = msg.position[2]
        self.robot.joint_angles[3] = msg.position[3]
        self.robot.joint_angles[4] = msg.position[4]
        
        # Update theta parameters
        self.robot.t_dh[0] = 0.
        self.robot.t_dh[1] = self.robot.joint_angles[0]
        self.robot.t_dh[2] = self.robot.joint_angles[1] - np.pi/2
        self.robot.t_dh[3] = self.robot.joint_angles[2] + np.pi/2
        self.robot.t_dh[4] = self.robot.joint_angles[3] + np.pi/2
        self.robot.t_dh[5] = 0.

if __name__ == '__main__':
    try:
        rospy.init_node('arm_node',anonymous=True)
        simulation = rospy.get_param('~simulation', False)
        node = ArmNode(simulation)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


