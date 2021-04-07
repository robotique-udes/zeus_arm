import numpy as np


class RobotArm():

	def __init__(self):

		# Robot geometry
		self.dof = 5

		# DH parameters are in order from world to end-effector        
		self.r_dh = np.array([0.,      0.,       0.73375, 0.5866,  0.,      0.,         0.]) 
		self.d_dh = np.array([0.15255, 0.06405,  0.,      0.,      0.01349,      0.25664,    0.01611])
		self.t_dh = np.array([0.,      0.,       0.,      0.,      0.,      0.,         0.])
		self.a_dh = np.array([0.,      np.pi/2,  0.,      0.,     -np.pi/2, 0.,         np.pi/2])

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
		
	def forward_kinematics(self,current_config):
		"""
		Calculates end effector position
		
		INPUTS
		current_config  : current robot joint space coordinates     (list 5x1)
		
		OUTPUTS
		r : current robot task coordinates                          (list 3x1)

		"""   
			  
		# Update t_dh with current config 
		self.t_dh[0] = 0.
		self.t_dh[1] = current_config[0]
		self.t_dh[2] = current_config[1] + np.pi/2
		self.t_dh[3] = current_config[2]
		self.t_dh[4] = current_config[3] - np.pi/2
		self.t_dh[5] = current_config[4]
		self.t_dh[6] = 0.


		# Extract transformation matrix
		WTG = self.dhs2T(self.r_dh,self.d_dh,self.t_dh,self.a_dh)
		
		# Assemble the end effector position vector
		r = np.array([WTG[0][3],WTG[1][3],WTG[2][3]]).T
	   
		return r

if __name__ == '__main__':
	robot = RobotArm()
	q1 = np.array([0., 0., 0., 0., 0.], dtype=np.float64)
	pos1 = robot.forward_kinematics(q1)
	print("Candle pos :")
	print(pos1)

	q2 = np.array([0., 0., 1.57, -1.57, 0.], dtype=np.float64)
	pos2 = robot.forward_kinematics(q2)
	print("square pos :")
	print(pos2)

	#q3 = np.array([0., 0., 1.57, 0., 0.], dtype=np.float64)
	#pos3 = robot.forward_kinematics(q3)
	#print("L pos :")
	#print(pos3)
