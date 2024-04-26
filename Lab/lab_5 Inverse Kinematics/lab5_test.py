#!/usr/bin/env python

import numpy as np
import math
from scipy.linalg import expm
import sys
import time

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
	theta = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
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

	yaw_WgripDegree = yaw_WgripDegree * PI / 180

	xgrip = -yWgrip
	ygrip = xWgrip
	zgrip = zWgrip
	print("xyzgrip", xgrip, ygrip, zgrip)

	xcen = xgrip - np.cos(yaw_WgripDegree) * l09
	ycen = ygrip - np.sin(yaw_WgripDegree) * l09
	zcen = zgrip

	# theta1
	thetas[0] = math.atan2(ycen , xcen) - math.asin((0.027 + l06) / math.sqrt(xcen ** 2 + ycen ** 2))        # Default value Need to Change
	# print(xcen, ycen, zcen, l02 - l04 + l06)
	# print(0.027 + l06)
    # l02 - l04 + l06 = 0.027 + l06 = 0.131

	# theta6
	thetas[5] = PI - (PI / 2 - thetas[0]) - yaw_WgripDegree     # Default value Need to Change

	l_long = math.sqrt((l06 + 0.027) ** 2 + l07 ** 2)
	theta_tri = math.atan2(l07, (l06 + 0.027))
 
	x3end = xcen - l_long * math.sin(theta_tri - thetas[0])
	y3end = ycen - l_long * math.cos(theta_tri - thetas[0])
	z3end = zcen + l10 + l08
	temp = math.sqrt((z3end - l01) ** 2 + x3end ** 2 + y3end ** 2)
	print("xyz3end", x3end, y3end, z3end)

	thetas[1]= - (math.atan2((z3end - l01) , math.sqrt(x3end ** 2 + y3end ** 2)) + math.acos((temp ** 2 + l03 ** 2 - l05 ** 2) / (2 * l03 * temp)))    # Default value Need to Change
	thetas[2]=  PI - math.acos((l03 ** 2 + l05 ** 2 - temp ** 2) / (2 * l03 * l05))      # Default value Need to Change
	thetas[3]= - thetas[1] - thetas[2]  # Default value Need to Change
	thetas[4]= - PI / 2      # Default value Need to Change
	

	print("theta1 to theta6: \n\n" + str(thetas) + "\n")
	
	return thetas
        

def main():
    if len(sys.argv) != 5:
        print("\n")
        print("rosrun lab5pkg_ik lab5_exec.py xWgrip yWgrip zWgrip yaw_WgripDegree \n")
    else:
        print("\nxWgrip: " + sys.argv[1] + ", yWgrip: " + sys.argv[2] + \
              ", zWgrip: " + sys.argv[3] + ", yaw_WgripDegree: " + sys.argv[4] + "\n")

        # Assuming lab_invk and lab_fk are defined elsewhere and correctly imported
        new_dest = lab_invk(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
        
        T = lab_fk(new_dest[0], new_dest[1], new_dest[2], new_dest[3], new_dest[4], new_dest[5])
        print("check T[0][3], T[1][3], T[2][3] with your input destination")
    
        print(f"T is \n\n{T}")
        print("\n\n")
        end_effct = T[0:3,3]
        print(f"end_effct is \n\n{end_effct}")
        print("\n\n")

# Ensure the following functions are defined and imported correctly:
# lab_invk() and lab_fk()

if __name__ == "__main__":
	try:
		main()
	except: 
		print("error")