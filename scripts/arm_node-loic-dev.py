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
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int16, Bool
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class ArmNode():

    def __init__(self, simulation=False):
        '''
        Node class for robot arm instance
        '''
        if simulation:
            rospy.logwarn("Initialized node in simulation...")
            # Watch out the joints number are inversed since the joint number are defined from the effector to the base
            self.j6_pub_sim = rospy.Publisher('/zeus_arm/joint_1_velocity_controller/command', Float64, queue_size=10)
            self.j5_pub_sim = rospy.Publisher('/zeus_arm/joint_2_velocity_controller/command', Float64, queue_size=10)
            self.j4_pub_sim = rospy.Publisher('/zeus_arm/joint_3_velocity_controller/command', Float64, queue_size=10)
            self.j3_pub_sim = rospy.Publisher('/zeus_arm/joint_4_velocity_controller/command', Float64, queue_size=10)
        else:
            rospy.loginfo("Initialized node")
        self.simulation = simulation

        # Zero speed publisher #10Hz
        self.zero_speed_pub = rospy.Publisher('/zeus_arm/zero_twist', Twist, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/10),self.zero_speed)
        rospy.loginfo("Setting zero speed publisher")

        # Section for inverse kinematic
        self.n_joints = 6
        self.n_joints_sim = 4
        self.robot = RoboticArm()
        self.calibration_done = True
        

        # Init subscripers
        rospy.Subscriber("/zeus_arm/cmd_vel_mux", Twist, self.send_cmd)
        rospy.Subscriber("/zeus_arm/joint_states", JointState ,self.update_joint_states)
        rospy.Subscriber("/zeus_arm/calibration_done", Bool ,self.update_calibration_status)


        # Init publishers std_msgs::Float64MultiArray
        self.joint_cmd_pub = rospy.Publisher('/zeus_arm/joint_commands', Float64MultiArray, queue_size=10)
        

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


    def send_cmd(self, twist):
        '''
        Publishes commands received from multiplexer, sends zeros if calibration is not done
        The command received is in twist since its easier for the twist mux package
        '''
        cmd = np.zeros((6), dtype=float)
        cmd[5], cmd[4], cmd[3] = twist.linear.x, twist.linear.y, twist.linear.z
        cmd[2], cmd[1], cmd[0] = twist.angular.x, twist.angular.y, twist.angular.z

        if not self.simulation:
            if self.calibration_done:
                self.joint_cmd_pub.publish(cmd)
            else:
                self.joint_cmd_pub.publish(np.zeros((self.n_joints)))

        else:
            if self.calibration_done:
                self.j6_pub_sim.publish(cmd[0])
                self.j5_pub_sim.publish(cmd[1])
                self.j4_pub_sim.publish(cmd[2])
                self.j3_pub_sim.publish(cmd[3])
            else:
                self.j6_pub_sim.publish(Float64())
                self.j5_pub_sim.publish(Float64())
                self.j4_pub_sim.publish(Float64())
                self.j3_pub_sim.publish(Float64())




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
        self.robot.joint_angles = np.array(msg.position)
        
        

if __name__ == '__main__':
    try:
        rospy.init_node('arm_node',anonymous=True)
        simulation = rospy.get_param('~simulation', False)
        node = ArmNode(simulation)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


