import numpy as np
from arm_class import RoboticArm

# TODO : 
	# - Recaracterize DH params with new design

def test_jacobian_matrix_home():
	# Create robot object
	robot = RoboticArm()

	# Set robot angles 
	robot.joint_angles[0] = 0.
	robot.joint_angles[1] = 0.
	robot.joint_angles[2] = 0.
	robot.joint_angles[3] = 0.
	robot.joint_angles[4] = 0.

	# Set reference matrix
	J_ref = np.zeros((6,robot.dof), dtype=np.float64)
	J_ref[0,:] = [0.,           1.44301602e-16,        1.44301602e-16,  3.22184979e-17,  0.]
	J_ref[1,:] = [-1.39407700,  0.,                    0.,              0.,              0.]
	J_ref[2,:] = [0.,           1.12846300,            5.89156000e-01,  1.31542000e-01,  0.]
	J_ref[3,:] = [0.,           0.,                    0.,              0.,              1.]
	J_ref[4,:] = [0., 		    1.,					   1.,		        1.,		         0.]
	J_ref[5,:] = [1.,		    0.,					   0.,		        0.,		         0.]

	# Get result
	J = robot.jacobian_matrix()

	np.testing.assert_allclose(J, J_ref, rtol=1e-5, atol=0)

def test_jacobian_matrix_home_extended():
	# Create robot object
	robot = RoboticArm()

	# Set robot angles 
	robot.joint_angles[0] = 0.
	robot.joint_angles[1] = np.pi/2
	robot.joint_angles[2] = -np.pi/2
	robot.joint_angles[3] = 0.
	robot.joint_angles[4] = 0.

	# Set reference matrix
	J_ref = np.zeros((6,robot.dof), dtype=np.float64)
	J_ref[0,:] = [0.,          -5.39307000e-01,       -7.21508010e-17, -1.61092489e-17,  0.]
	J_ref[1,:] = [3.23542e-01,  0.,                    0.,              0.,              0.]
	J_ref[2,:] = [0.,          -5.89156000e-01,       -5.89156000e-01, -1.31542000e-01,  0.]
	J_ref[3,:] = [0.,           0.,                    0.,              0.,              1.]
	J_ref[4,:] = [0., 		    1.,					   1.,		        1.,		         0.]
	J_ref[5,:] = [1.,		    0.,					   0.,		        0.,		         0.]
	print(J_ref)

	# Get result
	J = robot.jacobian_matrix()
	

	np.testing.assert_allclose(J, J_ref, rtol=1e-5, atol=0)

def test_forward_kinematics_home():
	# Create robot object
	robot = RoboticArm()

	# Set robot angles 
	robot.joint_angles[0] = 0.
	robot.joint_angles[1] = 0.
	robot.joint_angles[2] = 0.
	robot.joint_angles[3] = 0.
	robot.joint_angles[4] = 0.

	# Update DH parameters with joint positions
	robot.t_dh[0] = 0.
	robot.t_dh[1] = robot.joint_angles[0]
	robot.t_dh[2] = robot.joint_angles[1] - np.pi/2
	robot.t_dh[3] = robot.joint_angles[2] + np.pi/2
	robot.t_dh[4] = robot.joint_angles[3] + np.pi/2
	robot.t_dh[5] = 0.

	# Set reference vector
	ref = np.zeros((6,1), dtype=np.float64)
	ref[0] = 0.8544
	ref[1] = -0.00098
	ref[2] = 0.69342
	ref[3] = 0.
	ref[4] = np.pi/2
	ref[5] = 0.

	# Send joint config
	pos, _ = robot.forward_kinematics()

	# Verify near equality
	np.testing.assert_allclose(pos, ref, rtol=1e-5, atol=0)

def test_forward_kinematics_extended():
	# Create robot object
	robot = RoboticArm()

	# Set robot angles 
	robot.joint_angles[0] = 0.
	robot.joint_angles[1] = 0.
	robot.joint_angles[2] = 0.
	robot.joint_angles[3] = 0.
	robot.joint_angles[4] = 0.

	# Update DH parameters with joint positions
	robot.t_dh[0] = 0.
	robot.t_dh[1] = robot.joint_angles[0]
	robot.t_dh[2] = robot.joint_angles[1] - np.pi/2
	robot.t_dh[3] = robot.joint_angles[2] + np.pi/2
	robot.t_dh[4] = robot.joint_angles[3] + np.pi/2
	robot.t_dh[5] = 0.

	# Set reference vector
	ref = np.zeros((6,1), dtype=np.float64)
	ref[0] = 0.8544
	ref[1] = -0.00098
	ref[2] = 0.69342
	ref[3] = 0.
	ref[4] = np.pi/2
	ref[5] = 0.

	# Send joint config
	pos, _ = robot.forward_kinematics()

	# Verify near equality
	np.testing.assert_allclose(pos, ref, rtol=1e-5, atol=0)


