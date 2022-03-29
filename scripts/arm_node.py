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
from std_msgs.msg import Float64, Bool
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
            # Watch out the joints number are inversed since the joint number are defined from the effector to the base if you check on wires
            self.j1_pub_sim = rospy.Publisher('/zeus_arm/joint_1_velocity_controller/command', Float64, queue_size=10)
            self.j2_pub_sim = rospy.Publisher('/zeus_arm/joint_2_velocity_controller/command', Float64, queue_size=10)
            self.j3_pub_sim = rospy.Publisher('/zeus_arm/joint_3_velocity_controller/command', Float64, queue_size=10)
            self.j4_pub_sim = rospy.Publisher('/zeus_arm/joint_4_velocity_controller/command', Float64, queue_size=10)
            lambda_val = 1.
        else:
            rospy.loginfo("Initialized node")
            lambda_val = 0.1
        
        self.simulation = simulation

        # Section for inverse kinematic
        self.n_joints = 6
        self.n_joints_sim = 4
        self.n_joints_reverse_kin = 4
        self.robot = RoboticArm()
        self.calibration_done = True
        

        # Init subscripers
        rospy.Subscriber("/zeus_arm/cmd_vel_mux", Twist, self.send_cmd)
        rospy.Subscriber("/zeus_arm/linear_cmd", Float64MultiArray, self.linear_cmd)
        rospy.Subscriber("/zeus_arm/reverse_kin_cmd", Float64MultiArray, self.reverse_kin_cmd)
        rospy.Subscriber("/zeus_arm/joint_states", JointState ,self.update_joint_states)
        rospy.Subscriber("/zeus_arm/calibration_done", Bool ,self.update_calibration_status)

        # -------- Publishers ----------

        # Init publishers that sends joint speed command to arduino
        self.joint_cmd_pub = rospy.Publisher('/zeus_arm/joint_commands', Float64MultiArray, queue_size=10)

        # Publishers for the twist multiplexer
        self.revkin_cmd_pub = rospy.Publisher('/zeus_arm/reverse_kin_twist', Twist, queue_size=10)
        self.linear_cmd_pub = rospy.Publisher('/zeus_arm/linear_twist', Twist, queue_size=10)
        self.zero_speed_pub = rospy.Publisher('/zeus_arm/zero_twist', Twist, queue_size=10)

        self.effector_pos_pub = rospy.Publisher('/zeus_arm/effector_pos', Float64MultiArray, queue_size=10)

        # Zero speed publisher #10Hz
        rospy.Timer(rospy.Duration(1.0/10),self.zero_speed)
        rospy.loginfo("Setting zero speed publisher")
        

        # Initialize configurable params
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure("zeus_arm")

        # Add variables to ddr(name, description, default value, min, max, edit_method)        
        # Model Settings
        self.ddr.add_variable("lambda_gain", "float", lambda_val, 0., 10.)
        

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
            print('****Updating lambda gains', str(config[var_name]))
        return config


    def linear_cmd(self, msg):
        '''
        Receives command for linear jog (r_dot = array([x_dot,y_dot,z_dot])) and turns it into a joint speed command that is published into a Twist
        '''
        q_dot = self.robot.linear_jog_controller(msg.data)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = q_dot[0], q_dot[1], q_dot[2]
        cmd.angular.x, cmd.angular.y, cmd.angular.z = q_dot[3], 0., 0.
        self.linear_cmd_pub.publish(cmd)


    def reverse_kin_cmd(self, msg):
        '''
        Receives command for reverse kin (r = array([x,y,z])) and turns it into a joint speed command that is published into a Twist
        '''
        q_dot = self.robot.speed_controller(msg.data)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = q_dot[0], q_dot[1], q_dot[2]
        cmd.angular.x, cmd.angular.y, cmd.angular.z = q_dot[3], 0., 0.
        self.revkin_cmd_pub.publish(cmd)



    def send_cmd(self, twist):
        '''
        Publishes commands received from multiplexer, sends zeros if calibration is not done
        The command received is in twist since its easier for the twist mux package
        '''
        cmd = np.zeros((6), dtype=float)
        cmd[0], cmd[1], cmd[2] = twist.linear.x, twist.linear.y, twist.linear.z
        cmd[3], cmd[4], cmd[5] = twist.angular.x, twist.angular.y, twist.angular.z

        if not self.simulation:
            if self.calibration_done:
                self.joint_cmd_pub.publish(cmd)
            else:
                self.joint_cmd_pub.publish(np.zeros((self.n_joints)))

        else:
            if self.calibration_done:
                self.j1_pub_sim.publish(cmd[0])
                self.j2_pub_sim.publish(cmd[1])
                self.j3_pub_sim.publish(cmd[2])
                self.j4_pub_sim.publish(cmd[3])
            else:
                self.j1_pub_sim.publish(Float64())
                self.j2_pub_sim.publish(Float64())
                self.j3_pub_sim.publish(Float64())
                self.j4_pub_sim.publish(Float64())


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
        self.robot.joint_angles = np.array(msg.position)[:self.n_joints_reverse_kin]

        # Publish effector pos
        r, _ = self.robot.forward_kinematics(self.robot.joint_angles)
        self.effector_pos_pub.publish(Float64MultiArray(None, r))
        
        

if __name__ == '__main__':
    try:
        rospy.init_node('arm_node',anonymous=True)
        simulation = rospy.get_param('~simulation', False)
        node = ArmNode(simulation)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


