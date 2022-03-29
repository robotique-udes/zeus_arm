#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on January 27 2021
# @author: Loic Boileau loic.boileau@usherbrooke.ca

"""
@package zeus_arm

------------------------------------

ROS Node to teleoperate the arm

"""

import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int16, Float64MultiArray
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure



class TeleopNode():
    def __init__(self):
        '''
        Node class to teleoperate arm joint by joint
        '''
        rospy.init_node('teleop_arm', anonymous=False)

        self.curr_axis = 'x'
        self.list_axis = ['x', 'y', 'z']
        self.curr_joint = 0
        self.num_joints = 6
        self.last_change = time.time()

        # Teleop mode
        # 0 [default]: teleop 1 joint a a time
        # 1 : teleop multiple joints
        # 2 : linear jog
        self.mode = {0: "Teleop 1 joint at a time (Default)",
                     1: "Teleop multiple joints at same time",
                     2: "Linear jog"}
        self.act_mode = 0


        # Init publishers
        self.cmd_pub = rospy.Publisher('/zeus_arm/teleop_cmd', Twist, queue_size=10)
        self.cmd_pub_linear = rospy.Publisher('/zeus_arm/linear_cmd', Float64MultiArray, queue_size=10)
        self.calib_pub = rospy.Publisher('/zeus_arm/calib_cmd', Int16, queue_size=10)

        # Subscribe to joystick
        self.joy_sub = rospy.Subscriber('/joy_arm', Joy, self.joy_callback)

        # Initialize configurable params
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure("zeus_arm")

        # # Add variables to ddr(name, description, default value, min, max, edit_method)        
        # # Model Settings
        self.ddr.add_variable("linear_speed", "float", 0.3, 0, 10)
        self.ddr.add_variable("J1_speed", "float", 1, 0, 10)
        self.ddr.add_variable("J2_speed", "float", 1, 0, 10)
        self.ddr.add_variable("J3_speed", "float", 1, 0, 10)
        self.ddr.add_variable("J4_speed", "float", 1, 0, 10)
        self.ddr.add_variable("J5_speed", "float", 1, 0, 10)
        self.ddr.add_variable("J6_speed", "float", 1, 0, 10)


        # # Start Server
        self.ddr.start(self.dynamic_reconfigure_callback)
        rospy.sleep(1)


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
            self.__dict__[var_name] = config[var_name]
        return config


    def change_mode(self):
        '''
        Change current mode
        ----------
        '''
        if time.time() - self.last_change > 0.3:
            self.act_mode = (self.act_mode + 1)%len(self.mode)
            rospy.loginfo("Changed mode to :\n\t"+self.mode[self.act_mode])
            self.last_change = time.time()



    def change_joint(self, direction):
        '''
        Change current joint
        ----------
        Parameters
        ----------
        direction: int
            +1 or -1
        '''   
        if time.time() - self.last_change > 0.3:
            if direction >= 1:
                new_joint = (self.curr_joint + 1) % self.num_joints 
            else:
                new_joint = (self.curr_joint - 1) % self.num_joints

            self.curr_joint = new_joint
            self.last_change = time.time()
            rospy.loginfo("Changed controlled joint to : " + str(new_joint + 1))


    def change_axis(self):
        '''
        Change current mode
        ----------
        '''
        if time.time() - self.last_change > 0.3:
            self.curr_axis = self.list_axis[(self.list_axis.index(self.curr_axis)+1)%len(self.list_axis)]
            self.last_change = time.time()
            rospy.loginfo("Changed axis to : " + self.curr_axis)



    def send_cmd_teleop_simple(self, msg):
        # Create command structure
        cmd = Twist()
        joy_cmd = msg.axes[1]

        # Fill command
        if self.curr_joint == 0:
            cmd.linear.x = joy_cmd * self.J1_speed 
        elif self.curr_joint == 1:
            cmd.linear.y = joy_cmd * self.J2_speed 
        elif self.curr_joint == 2:
            cmd.linear.z = joy_cmd * self.J3_speed 
        elif self.curr_joint == 3:
            cmd.angular.x = joy_cmd * self.J4_speed 
        elif self.curr_joint == 4:
            cmd.angular.y = joy_cmd * self.J5_speed 
        elif self.curr_joint == 5:
            cmd.angular.z = joy_cmd * self.J6_speed

        return cmd

    def send_cmd_teleop_multiple(self, msg):
        # Create command structure
        cmd = Twist()

        # Fill command
        cmd.linear.x = msg.axes[0] * self.J1_speed 
        cmd.linear.y = msg.axes[1] * self.J2_speed 
        cmd.linear.z = msg.axes[3] * self.J3_speed 
        cmd.angular.x = msg.axes[4] * self.J4_speed 
        cmd.angular.y = 0 
        cmd.angular.z = 0

        return cmd

    def send_cmd_linear_jog(self, msg):
        cmd = Float64MultiArray(None, np.zeros(len(self.list_axis)))
        cmd.data[self.list_axis.index(self.curr_axis)] = msg.axes[1] * self.linear_speed
        return cmd


    def joy_callback(self, msg):
        '''
        Callback from joystick
        ----------
        Parameters
        ----------
        msg: Joy
            Message from joystick
            
            axes[0] -> Left joystick (left/right)
            axes[1] -> Left joystick (up/down)
            axes[2] -> LT(L2) (defaults to 1 and goes up to -1)
            axes[3] -> Right joystick (left/right)
            axes[4] -> Right joystick (up/down)
            axes[5] -> RT(R2) (defaults to 1 and goes up to -1)
            axes[6] -> Numpad (left/right)
            axes[7] -> Numpad (up/down)

            buttons[0] -> A(X)
            buttons[1] -> B(Circle)
            buttons[2] -> X(Triangle)
            buttons[3] -> Y(Square)
            buttons[4] -> LB(L1)
            buttons[5] -> RB(R1)
            buttons[6] -> Back(L2)
            buttons[7] -> Start(R2)
            buttons[8] -> Share
            buttons[9] -> Options
            buttons[10] -> Power
            buttons[11] -> L3
            buttons[12] -> R3
        '''

        # Check dead man switch
        if msg.buttons[5]:

            # Send calibration signal if share button is clicked
            if msg.buttons[8]:
                signal = Int16()
                signal.data = 1
                self.calib_pub.publish(signal)
                rospy.loginfo("Sent calib signal")

            # Change mode if options button is clicked
            if msg.buttons[9]:
                self.change_mode()

            # Depends on which mode is activated

            # Teleop 1 joint at a time (Default)
            if self.act_mode == 0: 
                if msg.buttons[0]:
                    self.change_joint(1)
                elif msg.buttons[2]:
                    self.change_joint(-1)

                cmd = self.send_cmd_teleop_simple(msg) 
                self.cmd_pub.publish(cmd)

            # Teleop multiple joints at same time
            elif self.act_mode == 1: 
                cmd = self.send_cmd_teleop_multiple(msg)
                self.cmd_pub.publish(cmd)

            # Linear jog
            elif self.act_mode == 2:
                # This mode sends a linear command directly to arm node instead of a twist
                # It will be converted and then sent as a twist in twist mux
                if msg.buttons[0] or msg.buttons[2]:
                    self.change_axis()

                cmd = self.send_cmd_linear_jog(msg)
                self.cmd_pub_linear.publish(cmd)
            

if __name__ == '__main__':
    try:
        node = TeleopNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
