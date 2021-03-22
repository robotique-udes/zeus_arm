#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on January 18 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca
#          Santiago Moya        santiago.moya@usherbrooke.ca


"""
@package zeus_arm

------------------------------------

ROS Node to teleoperate the arm joint by joint

"""

import time
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Joy, JointState
import numpy as np


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
        self.old_cmd = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cmd = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cmd_j23 = [0.0, 0.0]
        self.print_state()
        self.last_change = time.time()

        # Init publishers
        self.j1_pub = rospy.Publisher('/zeus_arm/joint_1_velocity_controller/command', Float32, queue_size=10)
        self.j2_pub = rospy.Publisher('/zeus_arm/joint_2_velocity_controller/command', Float32MultiArray, queue_size=10)
        self.j4_pub = rospy.Publisher('/zeus_arm/joint_4_velocity_controller/command', Float32, queue_size=10)
        self.j5_pub = rospy.Publisher('/zeus_arm/joint_5_velocity_controller/command', Float32, queue_size=10)

        # Init command loop 
        rospy.Timer(rospy.Duration(1.0/50), self.send_cmd_callback)

        # Subscribe to joystick
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Subscribe to arm state
        self.j1_sub = rospy.Subscriber('/zeus_arm/joint_1_state', Float32, self.j1_state_callback)
        self.j2_sub = rospy.Subscriber('/zeus_arm/joint_2_state', Float32, self.j2_state_callback)
        self.j4_sub = rospy.Subscriber('/zeus_arm/joint_4_state', Float32, self.j4_state_callback)
        self.j5_sub = rospy.Subscriber('/zeus_arm/joint_5_state', Float32, self.j5_state_callback)


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


    def j1_state_callback(self, msg):
        '''
        Callback from arm
        ----------
        Parameters
        ----------
        msg: JointState
            Message from arm (real)
        '''
        self.curr_pos[0] = msg.data

    def j2_state_callback(self, msg):
        '''
        Callback from arm
        ----------
        Parameters
        ----------
        msg: JointState
            Message from arm (real)
        '''
        self.curr_pos[1] = msg.data

    def j3_state_callback(self, msg):
        '''
        Callback from arm
        ----------
        Parameters
        ----------
        msg: JointState
            Message from arm (real)
        '''
        self.curr_pos[2] = msg.data

    def j4_state_callback(self, msg):
        '''
        Callback from arm
        ----------
        Parameters
        ----------
        msg: JointState
            Message from arm (real)
        '''
        self.curr_pos[3] = msg.data

    def j5_state_callback(self, msg):
        '''
        Callback from arm
        ----------
        Parameters
        ----------
        msg: JointState
            Message from arm (real)
        '''
        self.curr_pos[4] = msg.data

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

        # Rotating or linear joint?
        if self.curr_joint != 1 and self.curr_joint != 2:
            if self.curr_joint == 4 : 
                cmd = (msg.axes[1]) * 25
            else :
                cmd = (msg.axes[1]) * 5

            # To make it as valocity control
            self.cmd[self.curr_joint] = self.curr_pos[self.curr_joint] + 0.5*cmd
        else:
            self.cmd_j23 = np.array([self.curr_joint + 1, msg.axes[1]]).flatten().tolist()


    def send_cmd_callback(self, evt):
        '''
        Send commands to joints timer
        '''

        self.send_cmd()


    def send_cmd(self):
        '''
        Publishes commands to each joint
        '''
        if self.curr_joint == 0:
            if (abs(self.old_cmd[0] - self.cmd[0]) > 0.4):
                self.j1_pub.publish(self.cmd[0])
                self.old_cmd[0] = self.cmd[0]
            else:
                self.j1_pub.publish(self.old_cmd[0])

        if self.curr_joint == 1:
            data = Float32MultiArray()
            data.data = self.cmd_j23
            self.j2_pub.publish(data)
 
        if self.curr_joint == 2:
            data = Float32MultiArray()
            data.data = self.cmd_j23
            self.j2_pub.publish(data)

        if self.curr_joint == 3:
            if (abs(self.old_cmd[3] - self.cmd[3]) > 0.4):
                self.j4_pub.publish(self.cmd[3])
                self.old_cmd[3] = self.cmd[3]
            else:
                self.j4_pub.publish(self.old_cmd[3])

        if self.curr_joint == 4:
            if (abs(self.old_cmd[4] - self.cmd[4]) > 0.4):
                self.j5_pub.publish(self.cmd[4])
                self.old_cmd[4] = self.cmd[4]
            else:
                self.j5_pub.publish(self.old_cmd[4])



    def on_shutdown(self):
        '''
        Set commands to 0 at shutdown
        '''
        #self.cmd = [0.0, 0.0, 0.0, 0.0, 0.0]
        #self.send_cmd()



if __name__ == '__main__':
    try:
        node = JointTeleopNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

