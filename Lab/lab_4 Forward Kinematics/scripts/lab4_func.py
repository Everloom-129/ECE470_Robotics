#!/usr/bin/env python

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
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	S[0] = np.array([0,1,1,1,0,1])
	S[1] = np.array([0,0,0,0,-1,0])
	S[2] = np.array([1,0,0,0,0,0])
	S[3] = np.array([0,0,0,0,0.152,0])
	S[4] = np.array([0,0.152,0.152,0.152,0,0.152])
	S[5] = np.array([0,0,0.244,0.457,-0.131,0.542])


	M[0] = np.array([0,0,1,0.293])
	M[1] = np.array([0,1,0,-0.542])
	M[2] = np.array([-1,0,0,0.152])
	M[3] = np.array([0,0,0,1])
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
	theta = np.array([theta1 - 0.5 * PI, theta2, theta3, theta4 + 0.5 * PI, theta5, theta6])
	M,S = Get_MS()
	print(f"M is :\n{M}\n\n")
	print(f"S is :\n{S}\n\n")
	T = np.eye(4)
	for i in range(6):
		T = np.dot(T, expm(skew(S[:,i])*theta[i]))
	T = np.dot(T,M)
	print("Forward kinematics calculated:\n")
	print(f"T is :\n{T}\n\n")
	return T

def skew(s):
	return np.array([
		[0, -s[2], s[1], s[3]],
		[s[2], 0, -s[0], s[4]],
		[-s[1], s[0], 0, s[5]],
		[0,0,0,0]
	])