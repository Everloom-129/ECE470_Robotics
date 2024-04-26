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
	# =================== Your code starts here ====================#
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


	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# =========== Implement joint angle to encoder expressions here ===========

	# =================== Your code starts here ====================#
	theta = np.array([theta1 - 0.5 * PI, theta2, theta3, theta4 + 0.5 * PI, theta5, theta6])
	M,S = Get_MS()
	# print(f"M is :\n{M}\n\n")
	# print(f"S is :\n{S}\n\n")
	T = np.eye(4)

	for i in range(6):
		T = np.dot(T, expm(skew(S[:,i])*theta[i]))

	T = np.dot(T,M)
	
	print("Forward kinematics calculated:\n")
	# print(f"T is :\n{T}\n\n")
	return T



def skew(s):
	return np.array([
		[0, -s[2], s[1], s[3]],
		[s[2], 0, -s[0], s[4]],
		[-s[1], s[0], 0, s[5]],
		[0,0,0,0]
	])

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

    # theta1 to theta6
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	l01 = 0.152
	l02 = 0.120
	l03 = 0.244
	l04 = 0.093
	l05 = 0.213
	l06 = 0.104
	l07 = 0.085
	l08 = 0.092
	l09 = 0
	l10 = 0.07   # thickness of aluminum plate is around 0.01

	xgrip = xWgrip
	ygrip = yWgrip
	zgrip = zWgrip

	xcen = xWgrip - np.cos(yaw_WgripDegree) * l09
	ycen = yWgrip - np.sin(yaw_WgripDegree) * l09
	zcen = zWgrip

	# theta1
	thetas[0] = np.arctan2(ycen , xcen) - np.arcsin((0.027 + l06) / np.sqrt(xcen ** 2 + ycen ** 2))        # Default value Need to Change

	# theta6
	thetas[5] = PI - (PI / 2 - thetas[0]) - yaw_WgripDegree     # Default value Need to Change
 
	x3end = xcen - l07 * np.cos(thetas[0]) + (0.027 + l06) * np.sin(thetas[0])
	y3end = ycen - l07 * np.sin(thetas[0]) - (0.027 + l06) * np.cos(thetas[0])
	z3end = zcen + l10 + l08
	# 2
	temp = np.sqrt((zcen - l01) ** 2 + xcen ** 2)
	thetas[1]= - (np.arctan2((zcen - l01) , xcen) + np.arccos((temp ** 2 + l03 ** 2 - l05 ** 2) / (2 * l03 * temp)))    # Default value Need to Change
	thetas[2]= PI - np.arccos((l03 ** 2 + l05 ** 2 - temp ** 2) / (2 * l03 * l05))      # Default value Need to Change
	thetas[3]= - thetas[1] - thetas[2]  # Default value Need to Change
	thetas[4]= -PI/2      # Default value Need to Change

	print("theta1 to theta6: \n\n" + str(thetas) + "\n")
	
	
	thetas = np.radians(thetas)
	return thetas
