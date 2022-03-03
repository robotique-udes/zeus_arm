#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on 3 march 2022
# @author: Loic Boileau loic.boileau@usherbrooke.ca


"""
@package zeus_arm

------------------------------------

ROS Node to teleoperate the arm in caretsian position

"""

import time
import rospy
from std_msgs.msg import Int16

class status_publisher:

    def __init__(self):
        self.status_ctrl_joint_nb = 0

        #Subscribers
        self.ctrl_joint_nb_sub = rospy.Subscriber('/zeus_arm/status/ctrl_joint_nb', Int16, self.ctrl_joint_nb)


    def ctrl_joint_nb(self, joint_nb):
        if joint_nb != self.status_ctrl_joint_nb:
            rospy.loginfo("Changed controlled joint to : " + str(joint_nb.data + 1))
            self.status_ctrl_joint_nb = joint_nb


if __name__ == '__main__':
    try:
        rospy.init_node('show_status', anonymous=True)

        status_pub = status_publisher()
      
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

