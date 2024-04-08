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


def Get_MS():
    # 	# Fill in the correct values for S1~6, as well as the M matrix
    M = np.eye(4)
    S = np.zeros((6,6))

    # Home position of the end-effector 
    M = np.array([[1, 0, 0, 0.223],  # Orientation part
                  [0, 1, 0, -0.541],  # Adjust these values based on the UR3e's end-effector home position
                  [0, 0, 1, -0.249],  # and orientation in the zero configuration
                  [0, 0, 0, 1]])     # Homogeneous component

    # Screw axes for all six joints in the space frame
    # Assuming placeholder values, adjust based on actual geometry of UR3e
    S = np.array([[0, 0, 1, 0, 0, 0],  # S1 for joint 1
                  [0, 1, 0, -0.152, 0, 0],  # S2 for joint 2
                  [0, 1, 0, -0.152, 0, 0.244], # S3 for joint 3
                  [0, 1, 0, -0.152, 0, 0.457], # S4 for joint 4
                  [0, 0, 1, 0.104, 0.457, 0], # S5 for joint 5
                  [0, 1, 0, 0.104, 0.457, 0.213]]) # S6 for joint 6

    return M, S



"""
Function that calculates encoder numbers for each motor
"""
	# =========== Implement joint angle to encoder expressions here ===========

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




