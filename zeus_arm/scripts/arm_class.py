#!/usr/bin/env python

# -*- coding: utf-8 -*-

# Created on Thu May 28 14:40:02 2020
# @author: Santiago Moya		santiago.moya@usherbrooke.ca


"""
@package zeus_arm

------------------------------------

Rover's arm class

"""

import rospy
import numpy as np
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class RoboticArm() : 
	"""
	RoboticArm class
	
	5 DOF robot arm	
	"""
	def __init__(self):
		"""
		Constructor method

		"""
		# Robot geometry
		self.dof = 5
		self.l1 = 0.156175
		self.l2 = 0.265614
		self.l3 = 0.539307
		self.l4 = 0.457614
		self.l5 = 0.131542

		# DH parameters are in order from world to end-effector        
		self.r_dh = np.array([0.,      0.266,       0.53925, 0.45135,  0.,         0.])
		self.d_dh = np.array([0.05651, 0.09766,     0.,      0.,      -0.00098,   -0.13705])
		self.t_dh = np.array([0.,      0.,          0.,      0.,       0.,         0.])
		self.a_dh = np.array([0.,      -np.pi/2,    0.,      0.,      -np.pi/2,    0.])

		# Robot state
		self.ref_cmd = np.zeros((6,1), dtype=np.float64)
		self.joint_angles = np.zeros(5, dtype=np.float64)
		self.lambda_gain = 0.1


	def dh2T(self, r , d , theta, alpha ):
		"""  
		Creates a transformtation matrix based on DH parameters
		
		INPUTS
		r     : DH parameter            (float 1x1)
		d     : DH parameter            (float 1x1)
		theta : DH parameter            (float 1x1)
		alpha : DH parameter            (float 1x1)
		
		OUTPUTS
		T     : Transformation matrix   (float 4x4 (numpy array))
				
		"""
		T = np.zeros((4,4), dtype=np.float64)

		c = lambda ang : np.cos(ang)
		s = lambda ang : np.sin(ang)
		
		T[0][0] = c(theta)
		T[0][1] = -s(theta)*c(alpha)
		T[0][2] = s(theta)*s(alpha)
		T[0][3] = r*c(theta)
		
		T[1][0] = s(theta)
		T[1][1] = c(theta)*c(alpha)
		T[1][2] = -c(theta)*s(alpha)
		T[1][3] = r*s(theta)
		
		T[2][0] = 0
		T[2][1] = s(alpha)
		T[2][2] = c(alpha)
		T[2][3] = d
		
		T[3][0] = 0
		T[3][1] = 0
		T[3][2] = 0
		T[3][3] = 1
		
		# Sets extremely small values to zero
		for i in range(0,4):
			for j in range(0,4):
				if -1e-10 < T[i][j] < 1e-10:
					T[i][j] = 0
		
		return T
			

	def dhs2T(self, r , d , theta, alpha ):
		"""
		Creates transformation matrix from end effector base to world base
	
		INPUTS
		r     : DH parameters                               (float nx1)
		d     : DH parameters                               (float nx1)
		theta : DH parameters                               (float nx1)
		alpha : DH parameters                               (float nx1)
	
		OUTPUTS
		WTT : Transformation matrix from tool to world      (float 4x4 (numpy array))
	
		"""
		WTT = np.zeros((4,4), dtype=np.float64)
		XTY = np.zeros((4,4), dtype=np.float64) 
		INT = np.array([XTY])
		
		# Count the number of T matrices to calculate
		parametersCount = len(r)
		
		# Create array for matrices
		for y in range(0,parametersCount):
			INT = np.append(INT,[XTY],0)
			
		# Calculate each T matrix
		for x in range(0, parametersCount):
			INT[x] = self.dh2T(r[x],d[x], theta[x], alpha[x])
		
		# First time must be done outside loop, if not WTT will remain a zeros matrix
		WTT = INT[0]
		for i in range(0,parametersCount-1):
			WTT = WTT.dot(INT[i+1])    
		
		return WTT
		
	def forward_kinematics(self):
		"""
		Calculates end effector position
		
		OUTPUTS
		r : current robot task coordinates                          (list 3x1)

		"""   

		# Extract transformation matrix
		WTG = self.dhs2T(self.r_dh,self.d_dh,self.t_dh,self.a_dh)

		theta_x = np.arctan2(WTG[2][1],WTG[2][2])
		theta_y = np.arctan2(-WTG[2][0],np.sqrt(WTG[2][1]**2 + WTG[2][2]**2))
		theta_z= np.arctan2(WTG[1][0],WTG[0][0]) 
		
		# Assemble the end effector position vector
		r = np.zeros((6,1), dtype=np.float64)
		r[0] = WTG[0][3]
		r[1] = WTG[1][3]
		r[2] = WTG[2][3]
		r[3] = theta_x
		r[4] = theta_y
		r[5] = theta_z

		return r,WTG


	def jacobian_matrix(self):
		"""
		Calculates jacobian matrix 
		
		INPUTS
		current_config : current robot joint space coordinates (list 5x1)
		
		OUTPUTS
		Jac : jacobian matrix (float 3x5)                                           
		"""

		J = np.zeros((6,self.dof), dtype=np.float64)

		c = lambda ang : np.cos(ang)
		s = lambda ang : np.sin(ang)

		l1 = self.l1
		l2 = self.l2
		l3 = self.l3
		l4 = self.l4
		l5 = self.l5

		q = self.joint_angles

		J[0][0] = -(l2 + l3 * c(q[1]) + l4 *c((np.pi-q[1]) + q[2] + np.pi) + l5 * c((np.pi-q[1]) + q[2] + q[3] + np.pi)) * s(q[0])
		J[0][1] = (-l3 * s(q[1]) - l4 * s((np.pi-q[1]) + q[2] + np.pi) - l5 * s((np.pi-q[1]) + q[2] + q[3] + np.pi)) * c(q[0])
		J[0][2] = (-l4 * s((np.pi-q[1]) + q[2] + np.pi) - l5 * s((np.pi-q[1]) + q[2] + q[3] + np.pi)) * c(q[0])
		J[0][3] = (-l5 * s((np.pi-q[1]) + q[2] + q[3] + np.pi)) * c(q[0])
		J[0][4] = 0.

		J[1][0] = -(l2 + l3 * c(q[1]) + l4 *c((np.pi-q[1]) + q[2] + np.pi) + l5 * c((np.pi-q[1]) + q[2] + q[3] + np.pi)) * c(q[0])
		J[1][1] = -(-l3 * s(q[1]) - l4 * s((np.pi-q[1]) + q[2] + np.pi) - l5 * s((np.pi-q[1]) + q[2] + q[3] + np.pi)) * s(q[0])
		J[1][2] = -(-l4 * s((np.pi-q[1]) + q[2] + np.pi) - l5 * s((np.pi-q[1]) + q[2] + q[3] + np.pi)) * s(q[0])
		J[1][3] = -(-l5 * s((np.pi-q[1]) + q[2] + q[3] + np.pi)) * s(q[0])
		J[1][4] = 0.

		J[2][0] = 0.
		J[2][1] = l3 * c(q[1]) + l4 * c((np.pi-q[1]) + q[2] + np.pi) + l5 * c((np.pi-q[1]) + q[2] + q[3] + np.pi)
		J[2][2] = l4 * c((np.pi-q[1]) + q[2] + np.pi) + l5 * c((np.pi-q[1])+ q[2] + q[3] + np.pi)
		J[2][3] = l5 * c((np.pi-q[1]) + q[2] + q[3] + np.pi)
		J[2][4] = 0.

		J[3][0] = 0.
		J[3][1] = 0.
		J[3][2] = 0.
		J[3][3] = 0.
		J[3][4] = 1.

		J[4][0] = 0.
		J[4][1] = 1.
		J[4][2] = 1.
		J[4][3] = 1.
		J[4][4] = 0.

		J[5][0] = 1.
		J[5][1] = 0.
		J[5][2] = 0.
		J[5][3] = 0.
		J[5][4] = 0.

		# J[0][0] = -(l2 + l3 * s(q[1]) + l4 *c(q[1] + q[2] + np.pi) + l5 * c(q[1] + q[2] + q[3] + np.pi)) * s(q[0])
		# J[0][1] = (l3 * c(q[1]) - l4 * s(q[1] + q[2] + np.pi) - l5 * s(q[1] + q[2] + q[3] + np.pi)) * c(q[0])
		# J[0][2] = (-l4 * s(q[1] + q[2] + np.pi) - l5 * s(q[1] + q[2] + q[3] + np.pi)) * c(q[0])
		# J[0][3] = (-l5 * s(q[1] + q[2] + q[3] + np.pi)) * c(q[0])
		# J[0][4] = 0.

		# J[1][0] = -(l2 + l3 * s(q[1]) + l4 *c(q[1] + q[2] + np.pi) + l5 * c(q[1] + q[2] + q[3] + np.pi)) * c(q[0])
		# J[1][1] = -(l3 * c(q[1]) - l4 * s(q[1] + q[2] + np.pi) - l5 * s(q[1] + q[2] + q[3] + np.pi)) * s(q[0])
		# J[1][2] = -(-l4 * s(q[1] + q[2] + np.pi) - l5 * s(q[1] + q[2] + q[3] + np.pi)) * s(q[0])
		# J[1][3] = -(-l5 * s(q[1] + q[2] + q[3] + np.pi)) * s(q[0])
		# J[1][4] = 0.

		# J[2][0] = 0.
		# J[2][1] = -l3 * s(q[1]) + l4 * c(q[1] + q[2] + np.pi) + l5 * c(q[1] + q[2] + q[3] + np.pi)
		# J[2][2] = l4 * c(q[1] + q[2] + np.pi) + l5 * c(q[1]+ q[2] + q[3] + np.pi)
		# J[2][3] = l5 * c(q[1] + q[2] + q[3] + np.pi)
		# J[2][4] = 0.

		# J[3][0] = 0.
		# J[3][1] = 0.
		# J[3][2] = 0.
		# J[3][3] = 0.
		# J[3][4] = 1.

		# J[4][0] = 0.
		# J[4][1] = 1.
		# J[4][2] = 1.
		# J[4][3] = 1.
		# J[4][4] = 0.

		# J[5][0] = 1.
		# J[5][1] = 0.
		# J[5][2] = 0.
		# J[5][3] = 0.
		# J[5][4] = 0.

		return J 
			
		
	def get_joint_config(self):
		"""
		Returns robot joint configuration
		
		OUTPUTS
		q  : current robot configuration                            (list 5x1)

		"""
		return self.joint_angles
	

	def get_effector_pos(self):
		"""
		Returns current end effector position
		
		OUTPUTS
		end_effector  : current end effector coordinates     (list 3x1)

		"""
		return self.forward_kinematics(self.joint_angles) 


	def speed_controller(self):
		"""	
		Returns speed command to send to actuator
		
		OUTPUTS
		cmd_to_motors  : position command for motors     (list 5x1)
		"""
		q_dot = np.zeros((5,1), dtype = np.float64)
		J = self.jacobian_matrix()
		Jt = J.T
		I = np.identity(5)
		lambda2I = np.power(self.lambda_gain, 2) * I

		#change referential
		r , transf = self.forward_kinematics()
		rot = np.zeros((3,3), dtype = np.float64)
		rot = transf[0:3,0:3]
		ref_world = np.zeros((6,1), dtype = np.float64)
		ref_world[0:3] = np.dot(rot, self.ref_cmd[0:3])
		ref_world[3] = self.ref_cmd[3]
		ref_world[4] = self.ref_cmd[4]
		ref_world[5] = -self.ref_cmd[5] 

		q_dot = np.dot(np.dot(np.linalg.inv(np.dot(Jt,J) + lambda2I), Jt), ref_world)

		return q_dot.flatten().tolist()

		
