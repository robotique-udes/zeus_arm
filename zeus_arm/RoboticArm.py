# -*- coding: utf-8 -*-

# Created on Thu May 28 14:40:02 2020
# @author: santi


"""
@package robot_arm

------------------------------------

Package containing the rover's arm class
"""

class RoboticArm() : 
    """
    RoboticArm class
    
    5 DOF robot arm
    
    """
    
    def __init__(self):
        """
        Constructor method

        """
        # TODO : set exact robot dimensions
        self.l1 = 1 
        self.l2 = 2
        self.l3 = 3
        self.l4 = 4
        self.l5 = 5
        
    
    def jacobian_matrix(self, current_config):
        """
        Calculates jacobian matrix 
        
        INPUTS
        current_config : current robot joint space coordinates                  (list 5x1)
        
        OUTPUTS
        J : jacobian matrix                                                     (float 3x5)                                           
        """
              
        J = np.zeros((3,5))
        
        J[0,0] = 0
        J[0,1] = 0
        J[0,2] = 0       
        J[0,3] = 0
        J[0,4] = 0
        
        J[1,0] = 0
        J[1,1] = 0
        J[1,2] = 0       
        J[1,3] = 0
        J[1,4] = 0   
        
        J[2,0] = 0
        J[2,1] = 0 
        J[2,2] = 0        
        J[2,3] = 0
        J[2,4] = 0   
  
        
        return J
    

    def dh2T( r , d , theta, alpha ):
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
        
        T = np.zeros((4,4))

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
        
       
        # Sets extremely small valuer to zero
        for i in range(0,4):
            for j in range(0,4):
                if -1e-10 < T[i][j] < 1e-10:
                    T[i][j] = 0
        
        
        return T
            

    def dhs2T( r , d , theta, alpha ):
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
        
        WTT = np.zeros((4,4))
        XTY = np.zeros((4,4)) 
        INT = np.array([XTY])
        
        # count the number of T matrices to calculate
        parametersCount = len(r)
        
        # create array for matrices
        for y in range(0,parametersCount):
            INT = np.append(INT,[XTY],0)
            
        # calculate each T matrix
        for x in range(0, parametersCount):
            INT[x] = dh2T(r[x],d[x], theta[x], alpha[x])
        
        # first time must be done outside loop, if not WTT will remain a zeros matrix
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
        # angles 
        q1 = current_config[0]
        q2 = current_config[1]
        q3 = current_config[2]
        q4 = current_config[3]
        q5 = current_config[4]
              
        # r
        wra = 0
        arb = 0
        brc = self.l3
        crd = self.l4
        dre = 0
        erf = 0
        
        # d
        wda = self.l1
        adb = self.l2
        bdc = 0
        cdd = 0
        dde = 0 
        edf = self.l5
        
        # theta
        wta = 0
        atb = q1 + np.pi
        btc = q2
        ctd = q3
        dte = q4 - (np.pi/4)
        etf = 0
        
        # alpha
        waa = 0
        aab = np.pi/2
        bac = 0
        cad = 0
        dae = np.pi/2
        eaf = 0
           
        r_dh = np.array([wra,arb,brc,crd,dre,erf,frg]).T
        d_dh = np.array([wda,adb,bdc,cdd,dde,edf,fdg]).T
        theta_dh = np.array([wta,atb,btc,ctd,dte,etf,ftg]).T
        alpha_dh = np.array([waa,aab,bac,cad,dae,eaf,fag]).T
        
        # Extract transformation matrix
        WTG = dhs2T(r_dh,d_dh,theta_dh,alpha_dh)
        
        # Assemble the end effector position vector
        r = np.array([WTG[0][3],WTG[1][3],WTG[2][3]]).T
       
        return r
        
    def move_to_home():
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
    
    
    def get_joint_config():
        """
        Returns current end effector position
        
        OUTPUTS
        q  : current robot configuration                            (list 5x1)

        """
        
        # TODO : Code that reads current robot configuration for all joint motors
        q = 0
    
        return q
    
    def get_effector_pos():
        """
        Returns current end effector position
        
        OUTPUTS
        end_effector  : current end effector coordinates     (list 3x1)

        """
        q = get_joint_config()
        self.end_effector = forward_kinematics(q)
        
        return end_effector
        
    # TODO : controller function ?
