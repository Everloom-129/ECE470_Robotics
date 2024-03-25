#!/usr/bin/env python

'''

lab4pkg_fk/lab4_func.py

@brief: functions for computing forward kinematics using Product of Exponential (PoE) method
@author: Songjie Xiao
@date: Monday 2023/3/20

'''

import numpy as np
import math
from scipy.linalg import expm

PI = np.pi

"""
You may write some helper functions as you need
Use 'expm' for matrix exponential
Angles are in radian, distance are in meters.
"""

# def Get_MS():
# 	# =================== Your code starts here ====================#
# 	# Fill in the correct values for S1~6, as well as the M matrix
# 	M = np.eye(4)
# 	S = np.zeros((6,6))


	




# 	# ==============================================================#
# 	return M, S
def Get_MS():
    # Screw axes S1 to S6 in the robot's base frame
    S = np.array([
        [0,         0,         1,      0,      0,      0],
        [1,         0,         0,      -0.162, 0,      0.150],
        [1,         0,         0,      -0.162, 0,      0.420],
        [0,         1,         0,      0.094,  -0.420, 0],
        [1,         0,         0,      -0.162, 0,      0.726],
        [0,         1,         0,      0.094,  -0.726, 0]
    ]).T  # Transposed to match the (6,6) shape

    # Home configuration matrix M
    M = np.array([
        [1, 0, 0,     0.150],
        [0, 1, 0,     0],
        [0, 0, 1,     0.726],
        [0, 0, 0,     1]
    ])

    return M, S


"""
Function that calculates encoder numbers for each motor
"""

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    # Convert joint angles from degrees to radians
    theta = np.radians([theta1, theta2, theta3, theta4, theta5, theta6])
    
    # Get screw axes and home configuration matrix
    M, S = Get_MS()
    
    # Compute the exponential product
    T = np.eye(4)  # Start with identity matrix
    for i in range(6):
        T = np.dot(T, expm(skew(S[:, i]) * theta[i]))
    
    # Multiply by the home configuration matrix M
    T = np.dot(T, M)
    
    print("Forward kinematics calculated:\n")
    print(T, "\n")
    return T

def skew(s):
    """
    Takes a screw axis s and returns the corresponding skew-symmetric matrix for use in expm.
    """
    return np.array([
        [0,      -s[2],  s[1], s[3]],
        [s[2],   0,      -s[0],s[4]],
        [-s[1],  s[0],   0,    s[5]],
        [0,      0,      0,    0]
    ])


# def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

# 	# =========== Implement joint angle to encoder expressions here ===========
# 	print("Forward kinematics calculated:\n")

# 	# =================== Your code starts here ====================#









# 	# ==============================================================#
	
# 	print(str(T) + "\n")
# 	return T



