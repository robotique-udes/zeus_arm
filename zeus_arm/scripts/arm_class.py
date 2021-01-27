# -*- coding: utf-8 -*-

# Created on Thu May 28 14:40:02 2020
# @author: santi


"""
@package robot_arm

------------------------------------

Package containing the rover's arm class

"""

import rospy
import numpy as np

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

		# DH parameters are in order from world to end-effector        
		self.r_dh = np.array([0.,      0.,       0.73375, 0.5866,  0.,      0.]) 
		self.d_dh = np.array([0.15255, 0.06405,  0.,      0.,      0.,      0.25664])
		self.t_dh = np.array([0.,      0.,       0.,      0.,      0.,      0.])
		self.a_dh = np.array([0.,      np.pi/2,  0.,      0.,      np.pi/2, 0.])

		# Robot state
		self.ref_cmd = np.zeros((6,1),dtype=np.float32)
		self.J = np.zeros((6,self.dof), dtype=np.float32)
		self.joint_angles = np.zeros(5, dtype=np.float32)


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
		T = np.zeros((4,4), dtype=np.float32)

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
		WTT = np.zeros((4,4), dtype=np.float32)
		XTY = np.zeros((4,4), dtype=np.float32) 
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
		
	def forward_kinematics(self,current_config):
		"""
		Calculates end effector position
		
		INPUTS
		current_config  : current robot joint space coordinates     (list 5x1)
		
		OUTPUTS
		r : current robot task coordinates                          (list 3x1)

		"""   
		# Angles 
		q1 = self.joint_angles[0]
		q2 = self.joint_angles[1]
		q3 = self.joint_angles[2]
		q4 = self.joint_angles[3]
		q5 = self.joint_angles[4]
			  
		# Update t_dh with current config 
		self.t_dh[1] = self.joint_angles[0]
		self.t_dh[2] = self.joint_angles[1] + np.pi/2
		self.t_dh[3] = self.joint_angles[2]
		self.t_dh[4] = self.joint_angles[4] + np.pi/2
		
		# Extract transformation matrix
		WTG = self.dhs2T(self.r_dh,self.d_dh,self.t_dh,self.a_dh)
		
		# Assemble the end effector position vector
		r = np.array([WTG[0][3],WTG[1][3],WTG[2][3]]).T
	   
		return r


	def jacobian_matrix(self):
		"""
		Calculates jacobian matrix 
		
		INPUTS
		current_config : current robot joint space coordinates (list 5x1)
		
		OUTPUTS
		Jac : jacobian matrix (float 3x5)                                           
		"""
		Jac = np.zeros((6,self.dof), dtype=np.float32)      


		# Slice arrays for necessary parameters
		r = self.r_dh[:-1]
		d = self.d_dh[:-1]
		t = self.t_dh[:-1]
		a = self.a_dh[:-1]
		e_T_f = self.dh2T(self.r_dh[-1], self.d_dh[-1], self.t_dh[-1], self.a_dh[-1])

		for i in range(self.dof-1, -1, -1):

			# Slice arrays for necessary parameters
			r_loop = self.r_dh[i:]
			d_loop = self.d_dh[i:]
			t_loop = self.t_dh[i:]
			a_loop = self.a_dh[i:]
			
			# Step 1
			Ji = np.zeros(6, dtype=np.float32).T

			# Step 2
			n_T_e = self.dhs2T(r_loop, d_loop, t_loop, a_loop)
			T = np.matmul(n_T_e,e_T_f)

			# Step 3	
			pi = np.zeros(3, dtype=np.float32)
			pi = T[0:3,3]		
			vec = np.zeros(3, dtype=np.float32)
			vec[2] = 1
			Ji[0:3] = np.cross(vec.T,pi.T, axis = 0)
			Ji[3:] = vec

			# Step 4
			Jac[:,i] = Ji

			# Step 5
			rmat = np.zeros((6,6), dtype=np.float32)
			rmat[0:3,0:3] = T[0:3,0:3] 
			rmat[3:,3:] = T[0:3,0:3] 

			# Step 6
			Jac = rmat.dot(Jac)

		self.J = Jac

		
	def move_to_home(self):
		"""
		Moves robot arm to rest position
		
		"""
		q = [0,0,0,0,0,0] # set defined angles for home position
		move_to(q)
				
	def move_to(self,q):
		"""
		Moves robot arm to deired joint space configuartion
		
		INPUTS
		q  : desired robot joint space coordinates     (list 5x1)

		"""
		# TODO : code that sends joint space coordinates to all motors
	
	
	def get_joint_config(self):
		"""
		Returns robot joint configuration
		
		OUTPUTS
		q  : current robot configuration                            (list 5x1)

		"""
		# TODO : Code that reads current robot configuration for all joint motors
		q = np.zeros((5,1), dtype=np.float32)
	
		return q
	

	def get_effector_pos(self):
		"""
		Returns current end effector position
		
		OUTPUTS
		end_effector  : current end effector coordinates     (list 3x1)

		"""
		q = get_joint_config()
		self.end_effector = forward_kinematics(q)
		
		return end_effector


	def speed_controller(self):
		"""	
		Returns speed command to send to actuator
		
		OUTPUTS
		q_dot  : speed command for motors     (list 5x1)


		"""
		#rospy.loginfo("Inside control loop")
		#rospy.loginfo(self.ref_cmd)
		dt = 1/40
		q_dot = np.zeros((5,1), dtype = np.float32)
		J_inv = np.linalg.pinv(self.J) 
		q_dot = np.dot(J_inv,self.ref_cmd)
		q = self.joint_angles + q_dot*0.5

		cmd_to_motor = (q).flatten().tolist()
		return cmd_to_motor

	
		
