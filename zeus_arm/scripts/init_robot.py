#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on January 18 2021
# @author: Santiago Moya       santiago.moya@usherbrooke.ca


"""
@package zeus_arm

------------------------------------

Script that sets gazebo gravity to 0.

"""

import sys
import time
import rospy
import numpy as np 
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64  
from std_msgs.msg import Float64  
from std_srvs.srv import Empty
from std_msgs.msg import Float64

class InitSim():

    def __init__(self):
        '''
        Node class to set gazebo desired parameters
        '''

        # Get gazebo physics server
        rospy.wait_for_service('/gazebo/set_physics_properties')
        set_gravity = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

        t_step, max_update_rate, gravity, ode_config = self.get_params()
        set_physics_request = SetPhysicsPropertiesRequest()

        # Build and send request
        set_physics_request.time_step = t_step.data
        set_physics_request.max_update_rate = max_update_rate.data
        set_physics_request.gravity = gravity
        set_physics_request.ode_config = ode_config
        set_gravity(set_physics_request)

    	# Unpause Gazebo
    	rospy.loginfo("Unpausing Gazebo...")
    	rospy.wait_for_service('/gazebo/unpause_physics')
    	unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        resp = unpause_gazebo()
    	rospy.loginfo("Unpaused Gazebo.")


    def get_params(self): 
        '''
        Returns desired simulation parameters
        ''' 
        rospy.loginfo("inside params")
        time_step = Float64(0.001)
        max_update_rate = Float64(1000.0)
        gravity = Vector3()
        gravity.x = 0.0
        gravity.y = 0.0
        gravity.z = 0.0
        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_iters = 50
        ode_config.sor_pgs_w = 1.3
        ode_config.sor_pgs_rms_error_tol = 0.0
        ode_config.contact_surface_layer = 0.001
        ode_config.contact_max_correcting_vel = 0.0
        ode_config.cfm = 0.0
        ode_config.erp = 0.2
        ode_config.max_contacts = 20
        
        return time_step, max_update_rate, gravity, ode_config
        

if __name__ == '__main__':
    rospy.init_node('init_sim', anonymous=True)
    node = InitSim()

