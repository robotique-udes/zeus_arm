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
        self.cmd = Twist()
        #self.print_state()
        self.last_change = time.time()

        # Init publishers
        self.twist_pub = rospy.Publisher('/zeus_arm/cmd_vel', Twist, queue_size=10)

        # Init command loop 
        rospy.Timer(rospy.Duration(1.0/50), self.send_cmd_callback)

        # Subscribe to joystick
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)


    def joy_callback(self, msg):
        '''
        Callback from joystick
        ----------
        Parameters
        ----------
        msg: Joy
            Message from joystick
        '''

        # Save command
        self.cmd = Twist()

        # Left joystick up-down
        self.cmd.linear.x = msg.axes[1]
        # Left joystick left-right
        self.cmd.linear.y = msg.axes[0]
        # Right joystick up-down
        self.cmd.linear.z = msg.axes[4]
        # Num pad up-down
        self.cmd.angular.x = msg.axes[7]
        # Num pad left-right
        self.cmd.angular.y = msg.axes[6]
        # B button
        self.cmd.angular.z = msg.buttons[1]
        

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

