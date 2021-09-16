#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on January 27 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca
#          Santiago Moya        santiago.moya@usherbrooke.ca


"""
@package zeus_arm

------------------------------------

ROS Node to teleoperate the arm in caretsian position

"""

import time
import rospy
from sensor_msgs.msg import Joy, JointState
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from zeus_arm.msg import Command


class PosTeleopNode():
    def __init__(self):
        '''
        Node class to teleoperate arm joint by joint
        '''
        rospy.init_node('pos_teleop_arm', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Control mode, 1 = joint control, 2 = cartesian control
        self.ctrl_mode = 1
        self.curr_joint = 0
        self.num_joints = 5
        #self.cartesian_speed = 0.5

        # Init command
        self.cmd = Command()
        self.last_change = time.time()

        # Init publishers
        self.cmd_pub = rospy.Publisher('/zeus_arm/cmd_vel', Command, queue_size=10)

        # Init command loop 
        rospy.Timer(rospy.Duration(1.0/50), self.send_cmd_callback)

        # Subscribe to joystick
        self.joy_sub = rospy.Subscriber('/joy_arm', Joy, self.joy_callback)

        # Initialize configurable params
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure("zeus_arm")

        # # Add variables to ddr(name, description, default value, min, max, edit_method)        
        # # Model Settings
        self.ddr.add_variable("cartesian_speed", "float", 0.05, 0, 10)
        self.ddr.add_variable("J1_speed", "float", 0.1, 0, 10)
        self.ddr.add_variable("J2_speed", "float", 0.1, 0, 10)
        self.ddr.add_variable("J3_speed", "float", 0.1, 0, 10)
        self.ddr.add_variable("J4_speed", "float", 0.1, 0, 10)
        self.ddr.add_variable("J5_speed", "float", 0.5, 0, 10)
        # self.inputs = ['Joint', 'Cartesian']
        # self.input_enum = self.ddr.enum([ self.ddr.const("Cartesian", "int", 2, "Cartesian"),
        #                                  self.ddr.const("Joint", "int", 1, "Joint")],
        #                                 "Input enum for arm control mode")
        # self.ddr.add_variable("ctrl_mode", "desired control mode", 0, 0, 3, edit_method=self.input_enum)


        # # Start Server
        self.ddr.start(self.dynamic_reconfigure_callback)
        self.print_mode()
        self.print_joint()
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
                self.curr_joint = new_joint
                self.last_change = time.time()
            else:
                new_joint = (self.curr_joint - 1) % self.num_joints 
                self.curr_joint = new_joint
                self.last_change = time.time()
            self.print_joint()

    def change_mode(self):
        '''
        Changes arm control mode
        '''
        if time.time() - self.last_change > 0.3:
            # Send zero command
            self.cmd = Command()
            self.cmd.mode = self.ctrl_mode

            # Change control mode
            self.ctrl_mode +=1
            if self.ctrl_mode > 2:
                self.ctrl_mode = 1

            # Print activated control mode
            self.last_change = time.time()
            self.print_mode() 

    def print_joint(self):
        '''
        Prints current controlled joint
        '''
        rospy.loginfo("Controlling joint : %s", str(self.curr_joint+1))
        
    def print_mode(self):
        '''
        Prints current controlled joint
        '''
        if self.ctrl_mode ==1 : 
            rospy.loginfo("Joint control activated")
            self.print_joint()
        else:
            rospy.loginfo("Cartesian control activated")


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
            axes[2] -> LT
            axes[3] -> Right joystick (left/right)
            axes[4] -> Right joystick (up/down)
            axes[5] -> RT
            axes[6] -> Numpad (left/right)
            axes[7] -> Numpad (up/down)

            buttons[0] -> A
            buttons[1] -> B
            buttons[2] -> X
            buttons[3] -> Y
            buttons[4] -> LB
            buttons[5] -> RB
            buttons[6] -> Back
            buttons[7] -> Start
            buttons[8] -> Power
            buttons[9] -> L3
            buttons[10] -> R3
        '''
        # Change control mode if necessary
        if msg.buttons[7]:
            self.change_mode()

        # Save gripper command
        gripper = (msg.buttons[1] or -msg.buttons[2])

        # Verify active control mode
        if self.ctrl_mode == 1 : 
            if msg.buttons[0]:
                self.change_joint(-1)
            elif msg.buttons[3]:
                self.change_joint(1)

            # Only use left joystick to command
            cmd = msg.axes[1]
            

            # Create command structure
            self.cmd = Command()
            self.cmd.mode = self.ctrl_mode
            self.cmd.gripper_cmd = gripper

            # Fill command
            if self.curr_joint == 0:
                self.cmd.cmd.linear.x = cmd * self.J1_speed 
            elif self.curr_joint == 1:
                self.cmd.cmd.linear.y = cmd * self.J2_speed 
            elif self.curr_joint == 2:
                self.cmd.cmd.linear.z = cmd * self.J3_speed 
            elif self.curr_joint == 3:
                self.cmd.cmd.angular.x = cmd * self.J4_speed 
            elif self.curr_joint == 4:
                self.cmd.cmd.angular.y = cmd  * self.J5_speed          

        else:
            # Create command structure
            self.cmd = Command()
            self.cmd.mode = self.ctrl_mode
            self.cmd.gripper_cmd = gripper

            # Fill command
            self.cmd.cmd.linear.x = msg.axes[1] * self.cartesian_speed          
            self.cmd.cmd.linear.y = msg.axes[0] * self.cartesian_speed       
            self.cmd.cmd.linear.z = msg.axes[4] * self.cartesian_speed
            self.cmd.cmd.angular.x = msg.axes[7] * self.cartesian_speed         
            self.cmd.cmd.angular.y = msg.axes[6] * self.cartesian_speed
            self.cmd.cmd.angular.z = (msg.buttons[4] or -msg.buttons[5]) * self.J1_speed
        

    def send_cmd_callback(self, evt):
        '''
        Send commands to joints timer
        '''
        self.send_cmd()


    def send_cmd(self):
        '''
        Publishes commands
        '''
        self.cmd_pub.publish(self.cmd)


    def on_shutdown(self):
        '''
        Set commands to 0 at shutdown
        '''
        self.cmd = Command()
        self.cmd.mode = self.ctrl_mode
        self.send_cmd()



if __name__ == '__main__':
    try:
        node = PosTeleopNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

