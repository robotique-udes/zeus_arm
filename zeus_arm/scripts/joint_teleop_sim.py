#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on January 18 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_arm

------------------------------------

ROS Node to teleoperate the arm joint by joint

"""

import time
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy, JointState


class JointTeleopNode():
    def __init__(self):
        '''
        Node class to teleoperate arm joint by joint
        '''
        rospy.init_node('joint_teleop_arm', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Set 1st joint
        self.curr_joint = 0
        self.num_joints = 5 

        # Init commands in rad
        self.curr_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cmd = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.old_cmd = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.print_state()
        self.last_change = time.time()

        # Init publishers
        self.j1_pub = rospy.Publisher('/zeus_arm/joint_1_position_controller/command', Float64, queue_size=10)
        self.j2_pub = rospy.Publisher('/zeus_arm/joint_2_position_controller/command', Float64, queue_size=10)
        self.j3_pub = rospy.Publisher('/zeus_arm/joint_3_position_controller/command', Float64, queue_size=10)
        self.j4_pub = rospy.Publisher('/zeus_arm/joint_4_position_controller/command', Float64, queue_size=10)
        self.j5_pub = rospy.Publisher('/zeus_arm/joint_5_position_controller/command', Float64, queue_size=10)

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
        print("Controlling joint: " + str(self.curr_joint + 1))


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
            self.change_joint(-1)
        elif msg.buttons[3]:
            self.change_joint(1)

        # Save command
        cmd = msg.axes[1]
        
        if abs(cmd) <= 0.01:
            self.cmd[self.curr_joint] = self.old_cmd[self.curr_joint]
        else:
            self.cmd[self.curr_joint] = self.curr_pos[self.curr_joint] + 0.3*cmd
            self.old_cmd[self.curr_joint] = self.cmd[self.curr_joint]

    def send_cmd_callback(self, evt):
        '''
        Send commands to joints timer
        '''
        self.send_cmd()


    def send_cmd(self):
        '''
        Publishes commands
        '''
        self.j1_pub.publish(self.cmd[0])
        self.j2_pub.publish(self.cmd[1])
        self.j3_pub.publish(self.cmd[2])
        self.j4_pub.publish(self.cmd[3])
        self.j5_pub.publish(self.cmd[4])


    def on_shutdown(self):
        '''
        Set commands to 0 at shutdown
        '''
        self.cmd = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_cmd()



if __name__ == '__main__':
    try:
        node = JointTeleopNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

