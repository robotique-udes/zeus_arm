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
from std_msgs.msg import Float64, Bool, Int16
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
            lambda_val = 0.5
        else:
            rospy.loginfo("Initialized node")
            lambda_val = 0.1
            
        max_accel = 1. # m/s2
        
        self.simulation = simulation

        # --- reverse kinematic ---
        self.n_joints = 6
        self.n_joints_sim = 4
        self.n_joints_reverse_kin = 4
        self.robot = RoboticArm()

        self.calibration_done = True
        self.zero_speed_freq = 10. # Hz
        self.accel_watchdog_freq = 10. # Hz
        self.accel_watchdog_cmd = np.zeros(self.n_joints, dtype=float)
        self.previous_joint_cmd = np.zeros(self.n_joints, dtype=float)
        

        # -------- Subscribers ----------

        rospy.Subscriber("/zeus_arm/vel_watchdog_cmd", Float64MultiArray, self.send_cmd)
        # The callback for accel_watchdog is on a timer loop to be able to calculate the acceleration
        # this subscriber is just there to set new value of cmd_vel_mux asynchroniously
        rospy.Subscriber("/zeus_arm/cmd_vel_mux", Twist, self.accel_watchdog_sub)
        rospy.Subscriber("/zeus_arm/linear_cmd", Float64MultiArray, self.linear_cmd)
        rospy.Subscriber("/zeus_arm/reverse_kin_cmd", Float64MultiArray, self.reverse_kin_cmd)
        rospy.Subscriber("/zeus_arm/joint_positions", Float64MultiArray ,self.update_joint_states)

        # -------- Publishers ----------

        # Init publishers that sends joint speed command to arduino
        self.joint_cmd_pub = rospy.Publisher('/zeus_arm/joint_commands', Float64MultiArray, queue_size=10)

        # Init publisher that checks for a limit acceleration
        self.accel_watchdog_cmd_pub = rospy.Publisher('/zeus_arm/vel_watchdog_cmd', Float64MultiArray, queue_size=10)

        # Publishers for the twist multiplexer
        self.revkin_cmd_pub = rospy.Publisher('/zeus_arm/reverse_kin_twist', Twist, queue_size=10)
        self.linear_cmd_pub = rospy.Publisher('/zeus_arm/linear_twist', Twist, queue_size=10)
        self.zero_speed_pub = rospy.Publisher('/zeus_arm/zero_twist', Twist, queue_size=10)

        self.effector_pos_pub = rospy.Publisher('/zeus_arm/effector_pos', Float64MultiArray, queue_size=10)
        
        # -------------------------------

        # Zero speed publisher #10Hz
        rospy.Timer(rospy.Duration(1.0/self.zero_speed_freq), self.zero_speed)
        rospy.loginfo("Setting zero_speed publisher")

        # Joint command to arduino #10Hz
        rospy.Timer(rospy.Duration(1.0/self.accel_watchdog_freq), self.accel_watchdog)
        rospy.loginfo("Setting accel_watchdog publisher")


        # ------ Initialize configurable params -------
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure("zeus_arm")

        # Add variables to ddr(name, description, default value, min, max, edit_method)        
        # Model Settings
        # setting dict:
        self.config = {"lambda_gain": lambda_val, "max_accel": max_accel}
        self.ddr.add_variable("lambda_gain", "float", self.config["lambda_gain"], 0., 10.)
        self.ddr.add_variable("max_accel", "float", self.config["max_accel"], 0., 10.)

        # Update internal variables values
        self.__dict__.update(self.config)
        self.dynamic_reconfigure_callback(self.config, None)

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
            if var_name == "lambda_gain":
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


    def accel_watchdog_sub(self, twist):
        '''
        Writes new cmd_vel_mux value as an array. This is just to set new value, 
        the accel_watchdog thread is the function below (accel_watchdog). If its not calibrated sends zeros.
        The command received is in twist since its easier for the twist mux package
        '''
        cmd = np.zeros(self.n_joints, dtype=float)
        if self.calibration_done:
            cmd[0], cmd[1], cmd[2] = twist.linear.x, twist.linear.y, twist.linear.z
            cmd[3], cmd[4], cmd[5] = twist.angular.x, twist.angular.y, twist.angular.z

        self.accel_watchdog_cmd = cmd


    def accel_watchdog(self, event):
        '''
        Check if the desired velocity ask for an accepted acceleration
        If not rewrites it for a velocity with max accel
        '''
        cmd = self.accel_watchdog_cmd
        dt = (1/self.accel_watchdog_freq)

        for i in range(len(cmd)):
            a = (self.accel_watchdog_cmd[i] - self.previous_joint_cmd[i]) / dt

            if (a > self.max_accel) and (a > 0):
                cmd[i] = (self.max_accel*dt) + self.previous_joint_cmd[i]

            if (a < self.max_accel) and (a < 0):
                cmd[i] = (-self.max_accel*dt) + self.previous_joint_cmd[i]

        self.accel_watchdog_cmd_pub.publish(Float64MultiArray(None, cmd))
        self.previous_joint_cmd = cmd


    def send_cmd(self, cmd):
        '''
        Send cmd to joint in Arduino
        '''

        if not self.simulation:
            self.joint_cmd_pub.publish(cmd)

        else:
            cmd = cmd.data
            self.j1_pub_sim.publish(cmd[0])
            self.j2_pub_sim.publish(cmd[1])
            self.j3_pub_sim.publish(cmd[2])
            self.j4_pub_sim.publish(cmd[3])
        


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
        self.robot.joint_angles = np.array(msg.data)[:self.n_joints_reverse_kin]

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


