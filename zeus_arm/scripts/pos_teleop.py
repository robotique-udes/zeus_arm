#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on January 27 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_arm

------------------------------------

ROS Node to teleoperate the arm in caretsian position

"""

import time
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist


class PosTeleopNode():
    def __init__(self):
        '''
        Node class to teleoperate arm joint by joint
        '''
        rospy.init_node('pos_teleop_arm', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Set 1st axis
        self.curr_axis = 0
        self.num_axis = 6 
        self.axes = ['x', 'y', 'z', 'qx', 'qy', 'qz']

        # Init commands in rad
        self.curr_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cmd = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.print_state()
        self.last_change = time.time()

        # Init publishers
        self.twist_pub = rospy.Publisher('/zeus_arm/cmd_vel', Twist, queue_size=10)

        # Init command loop 
        rospy.Timer(rospy.Duration(1.0/10), self.send_cmd_callback)

        # Subscribe to joystick
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Subscribe to arm state
        self.arm_sub = rospy.Subscriber('/zeus_arm/joint_states', JointState, self.state_callback)


    def print_state(self):
        '''
        Prints state for user.
        '''
        print("Controlling axis: " + self.axes[self.curr_axis])


    def change_axis(self, direction):
        '''
        Change current axis
        ----------
        Parameters
        ----------
        direction: int
            +1 or -1
        '''
        if time.time() - self.last_change > 0.3:
            if direction >= 1:
                new_axis = (self.curr_axis + 1) % self.num_axis 
                self.curr_axis = new_axis
                self.last_change = time.time()
            else:
                new_axis = (self.curr_axis - 1) % self.num_axis 
                self.curr_axis = new_axis
                self.last_change = time.time()
            self.print_state()


    def state_callback(self, msg):
        '''
        Callback from arm
        ----------
        Parameters
        ----------
        msg: JointState
            Message from arm (sim or real)
        '''
        self.curr_pos = list(msg.position)


    def joy_callback(self, msg):
        '''
        Callback from joystick
        ----------
        Parameters
        ----------
        msg: Joy
            Message from joystick
        '''
        # Change joint if necessary
        if msg.buttons[0]:
            self.change_axis(-1)
        elif msg.buttons[3]:
            self.change_axis(1)

        # Save command
        cmd = msg.axes[1]
        self.cmd = Twist()
        if self.curr_axis == 0:
            self.cmd.linear.x = cmd
        elif self.curr_axis == 1:
            self.cmd.linear.y = cmd
        elif self.curr_axis == 2:
            self.cmd.linear.z = cmd
        elif self.curr_axis == 3:
            self.cmd.angular.x = cmd
        elif self.curr_axis == 4:
            self.cmd.angular.y = cmd
        elif self.curr_axis == 5:
            self.cmd.angular.z = cmd


    def send_cmd_callback(self, evt):
        '''
        Send commands to joints timer
        '''
        self.send_cmd()


    def send_cmd(self):
        '''
        Publishes commands
        '''
        self.twist_pub.publish(self.cmd)


    def on_shutdown(self):
        '''
        Set commands to 0 at shutdown
        '''
        self.cmd = Twist()
        self.send_cmd()



if __name__ == '__main__':
    try:
        node = PosTeleopNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

