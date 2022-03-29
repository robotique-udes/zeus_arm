#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Created on Thu May 28 14:40:02 2020
# @author: Loic Boileau loic.boileau@usherbrooke.ca


"""
@package zeus_arm

------------------------------------

Rover's arm class

"""

import rospy
import numpy as np
from numpy import cos as c, sin as s, pi
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class RoboticArm() : 
	"""
	RoboticArm class
	
	4 DOF robot arm	
	"""
	def __init__(self):
		"""
		Constructor method

		"""
		## Robot geometry
		self.dof = 4
		
		# Joint 6 (base rotation)
		self.l1h = 0.13633 #Height of joint 5 with respect to joint 6
		self.l1d = 0.266 #Length of joint 5 with respect to joint 6

		# Joint 5 (shoulder)
		self.l2 = 0.509 # m

		# Joint 4 (elbow)
		#Since elbow has an offset, the length is calculated with pythagore
		#sqrt(h^2+d^2) => sqrt(0.02851^2+0.45365^2)
		self.l3 = 0.4545  # m

		# Joint 3 (wrist)
		self.l4 = 0.2046 # m

		# Robot angle between joint 3 and 4 (phi)
		#phi = atan(0.02851/0.45365)
		self.phi = 0.06276 # rad

		# Robot state
		self.ref_cmd = np.zeros((6,1), dtype=np.float64)
		self.joint_angles = np.zeros(4, dtype=np.float64)
		self.lambda_gain = 0.1 # TO CHANGE: dynamic parameter


	def write_joint_angles(self, angles):
		self.joint_angles = angles


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

		T = np.array([
			[c(theta), -s(theta)*c(alpha), s(theta)*s(alpha), r*c(theta)],
			[s(theta), c(theta)*c(alpha), -c(theta)*s(alpha), r*s(theta)],
			[0, s(alpha), c(alpha), d],
			[0, 0, 0, 1]
		])
		
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
		WTT = np.identity(4)
		for r, d, theta, alpha in zip(r, d, theta, alpha):
			WTT = np.dot(WTT, self.dh2T(r, d, theta, alpha))   
		
		return WTT

	def forward_kinematics(self, q):
		"""

		Parameters
		----------
		q : float 4x1
			Joint space coordinates

		Returns
		-------
		r : float 3x1 
			Effector (x,y,z) position

		"""
		r = np.zeros((3,1))
		
		# Parametres DH trouve a partir du CAD du robot
		DH_par = {
			'd' : 		np.array([ self.l1h,	0,  			0, 			  			0]),
			'r' : 		np.array([ self.l1d,  	self.l2,  		self.l3, 				self.l4]),
			'theta' : 	np.array([ q[0], 		(pi/2)-q[1],	self.phi-q[2]-(pi/2),	-q[3]-self.phi]),
			'alpha' : 	np.array([ (pi/2),		0, 				0,  			  		0])
		}

		WTT = self.dhs2T(
			r = DH_par['r'],
			d = DH_par['d'],
			theta = DH_par['theta'],
			alpha = DH_par['alpha']
		)
		
		r = WTT[:3,-1]
		
		return r, WTT


	def jacobian_matrix(self, q):
		"""
		Calculates jacobian matrix 
		
		INPUTS
		current_config : current robot joint space coordinates (list 5x1)
		
		OUTPUTS
		Jac : jacobian matrix (float 3x5)                                           
		"""
		l1h, l1d, l2, l3, l4, phi = self.l1h, self.l1d, self.l2, self.l3, self.l4, self.phi

		q1, q2, q3, q4 = q

		# Used wolframalpha to calculate this (Copy paste it inside the math input)
		# {{cos\(40)q_1\(41),0,sin\(40)q_1\(41),l_12*cos\(40)q_1\(41)},{sin\(40)q_1\(41),0,-cos\(40)q_1\(41),l_12*sin\(40)q_1\(41)},{0,1,0,l_11},{0,0,0,1}} . {{cos\(40)Divide[π,2]-q_2\(41),-sin\(40)Divide[π,2]-q_2\(41),0,l_2*cos\(40)Divide[π,2]-q_2\(41)},{sin\(40)Divide[π,2]-q_2\(41),cos\(40)Divide[π,2]-q_2\(41),0,l_2*sin\(40)Divide[π,2]-q_2\(41)},{0,0,1,0},{0,0,0,1}}.{{cos\(40)Divide[-π,2]-q_3+phi\(41),-sin\(40)Divide[-π,2]-q_3+phi\(41),0,l_3*cos\(40)Divide[-π,2]-q_3+phi\(41)},{sin\(40)Divide[-π,2]-q_3+phi\(41),cos\(40)Divide[-π,2]-q_3+phi\(41),0,l_3*sin\(40)Divide[-π,2]-q_3+phi\(41)},{0,0,1,0},{0,0,0,1}}.{{cos\(40)-q_4-phi\(41),-sin\(40)-q_4-phi\(41),0,l_4*cos\(40)-q_4-phi\(41)},{sin\(40)-q_4-phi\(41),cos\(40)-q_4-phi\(41),0,l_4*sin\(40)-q_4-phi\(41)},{0,0,1,0},{0,0,0,1}}
		
		## for X :
		# simplified
		# cos(q_1)(cos(-q_2-q_3+phi)(l_4cos(q_4+phi)+l_3)+(l_4sin(-q_2-q_3+phi)sin(q_4+phi)+l_2sin(q_2)+l_12))

		dx_dq1 = -s(q1)*(l3*c(-q2-q3+phi)+l2*s(q2)+l4*c(q2+q3+q4)+l1d)
		dx_dq2 = c(q1)*(l3*s(-q2-q3+phi)-l4*s(q2+q3+q4)+l2*c(q2))
		dx_dq3 = c(q1)*(l3*s(-q2-q3+phi)-l4*s(q2+q3+q4))
		dx_dq4 = -l4*s(q2+q3+q4)*c(q1)

		## for Y :
		# simplified
		# sin(q_1) (l_3 cos(-q_2 - q_3 + ϕ) + l_2 sin(q_2) + l_4 cos(q_2 + q_3 + q_4) + l_12)

		dy_dq1 = c(q1)*(l3*c(-q2-q3+phi)+l2*s(q2)+l4*c(q2+q3+q4)+l1d)
		dy_dq2 = s(q1)*(l3*s(-q2-q3+phi)-l4*s(q2+q3+q4)+l2*c(q2))
		dy_dq3 = s(q1)*(l3*s(-q2-q3+phi)-l4*s(q2+q3+q4))
		dy_dq4 = -l4*s(q1)*s(q2+q3+q4)

		## for Z :
		# simplified
		# l_3 sin(-q_2 - q_3 + ϕ) - l_4 sin(q_2 + q_3 + q_4) + l_2 cos(q_2) + l_11
		
		dz_dq1 = 0
		dz_dq2 = -l3*c(-q2-q3+phi)-l2*s(q2)-l4*c(q2+q3+q4)
		dz_dq3 = -l3*c(-q2-q3+phi)-l4*c(q2+q3+q4)
		dz_dq4 = -l4*c(q2+q3+q4)

		J = np.array([
			[dx_dq1, dx_dq2, dx_dq3, dx_dq4],
			[dy_dq1, dy_dq2, dy_dq3, dy_dq4],
			[dz_dq1, dz_dq2, dz_dq3, dz_dq4]
		])

		return J 
		

	def speed_controller(self, r_d, qv=None):
		"""	
		Returns speed command to send to actuator
		INPUT
		r_d 	: desired position of effector
		qv		: secondary objective for joint poisition

		OUTPUTS
		q_dot  : speed command for motors     (list 5x1)
		"""
		q = self.joint_angles
		J = self.jacobian_matrix(q)

		r, T = self.forward_kinematics(q)
		r_error = r_d - r
		r_dot = self.lambda_gain * r_error
		#print(r_dot)
		J_pseudo_inv = np.matmul(J.T, np.linalg.inv(np.dot(J,J.T)))

		q_dot = np.matmul(J_pseudo_inv, r_dot)

		#print(q_dot)

		# Add null space projection (qv is secondary-objective):
		if qv:
			I = np.identity(len(qv))
			# crop J second dimension
			J = J[:, len(qv):]
			# crop J_pseudo first dimension
			J_pseudo_inv = J_pseudo_inv[len(qv):, :]
			
			q_dot = q_dot + np.matmul(np.matmul(I , J_pseudo_inv*J), qv) 

		return q_dot

	def linear_jog_controller(self, rdot_d):
		"""	
		Returns speed command to send to actuator
		INPUT
		rdot_d 	: desired speed of effector in base frame

		OUTPUTS
		q_dot  : speed command for motors     (list 4x1)
		"""
		#Add change of reference if linear jog from effector frame

		q = self.joint_angles
		J = self.jacobian_matrix(q)

		J_pseudo_inv = np.matmul(J.T, np.linalg.inv(np.dot(J,J.T)))

		q_dot = np.matmul(J_pseudo_inv, rdot_d)

		return q_dot

		
