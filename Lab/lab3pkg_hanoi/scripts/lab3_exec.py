#!/usr/bin/env python

'''
ECE470 Lab3
Group# SandArm
Jie Wang, Xuan Tang, Shenghua Ye
03/17/2024

We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import copy
import time
import rospy
import numpy as np
from lab3_header import *

# 20Hz
SPIN_RATE = 20 

# UR3 home location
"""
TODO: Define your own Home position
"""
home = np.radians([-81.94,-114.59,-102.71,-12.07,90.34,19.33])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

############## Your Code Start Here ##############

"""
TODO: Definition of position of our tower in Q
"""

Q = np.zeros((3,4,6))
Q[0,0] = np.radians([-81.73,-125.69,-106.78,-37.44,90.29,356.47])
Q[0,1] = np.radians([-81.83,-122.53,-105.69,-41.70,90.31,356.35])
Q[0,2] = np.radians([-81.64,-119.07,-105.25,-45.59,90.32,356.53])
Q[0,3] = np.radians([-81.63,-110.23,-96.98,-62.70,90.36,356.43])
Q[1,0] = np.radians([-68.50,-124.51,-109.76,-35.64,90.28,9.71])
Q[1,1] = np.radians([-68.61,-121.41,-108.49,-40.02,90.30,9.58])
Q[1,2] = np.radians([-68.25,-117.77,-107.53,-44.61,90.32,9.92])
Q[1,3] = np.radians([-68.88,-109.64,-101.87,-58.41,90.36,9.21])
Q[2,0] = np.radians([-56.42,-126.69,-105.64,-37.58,90.28,21.78])
Q[2,1] = np.radians([-56.15,-122.37,-105.23,-42.32,90.31,22.03])
Q[2,2] = np.radians([-56.15,-119.15,-103.80,-46.96,90.33,22.00])
Q[2,3] = np.radians([-56.15,-109.84,-96.94,-63.13,90.36,21.90])


############### Your Code End Here ###############
 
############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback function for getting the state of suction cup
Whenever /ur_hardware_interface/io_states publishes info, this callback function is called.
"""
def gripper_input_callback(msg):
	global current_io_0
	global digital_in_0
	global analog_in_0
	
	digital_in_0 = msg.digital_in_states
	analog_in_0 = msg.analog_in_states
	current_io_0 = msg.flag_states
 
############### Your Code End Here ###############
 

"""
Whenever /joint_states publishes info, this callback function is called.
"""
def position_callback(msg):

	global thetas
	global current_position
	global current_position_set

	thetas[0] = msg.position[0]
	thetas[1] = msg.position[1]
	thetas[2] = msg.position[2]
	thetas[3] = msg.position[3]
	thetas[4] = msg.position[4]
	thetas[5] = msg.position[5]

	current_position[0] = thetas[0]
	current_position[1] = thetas[1]
	current_position[2] = thetas[2]
	current_position[3] = thetas[3]
	current_position[4] = thetas[4]
	current_position[5] = thetas[5]

	current_position_set = True


############## Your Code Start Here ##############

"""
TODO: define a function for ROS Publisher to publish your message to the Topic "ur3e_driver_ece470/setio",
so that we can control the state of suction cup.
"""
def gripper(pub_setio, io_0):
	msg = Digital()
	msg.pin = 0
	msg.state = io_0
	pub_setio.publish(msg)
	time.sleep(1)


############### Your Code End Here ###############
 
def move_arm(pub_setjoint, dest):
	msg = JointTrajectory()
	msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint","wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
	point = JointTrajectoryPoint()
	point.positions = dest
	point.time_from_start = rospy.Duration(2)
	msg.points.append(point)
	pub_setjoint.publish(msg)
	time.sleep(2.5)

############## Your Code Start Here ##############
"""
TODO: function to move block from start to end
"""
### Hint: Use the Q array to map out your towers by location and height.

def move_block(pub_setjoint, pub_setio, start_loc, start_height, end_loc, end_height):
    global Q

    # Turn off suction before starting
    gripper(pub_setio, suction_off)
    time.sleep(1)

    # Move above the start block
    move_arm(pub_setjoint, Q[start_loc][3])
    time.sleep(1)

    # Lower to the start block
    move_arm(pub_setjoint, Q[start_loc][start_height])
    time.sleep(1)

    # Turn on suction to pick up the block
    gripper(pub_setio, suction_on)
    time.sleep(1)

    # Move up with the block
    move_arm(pub_setjoint, Q[start_loc][3])
    time.sleep(1)

    # Move above the end block
    move_arm(pub_setjoint, Q[end_loc][3])
    time.sleep(1)

    # Lower to the end block position
    move_arm(pub_setjoint, Q[end_loc][end_height])
    time.sleep(1)

    # Turn off suction to release the block
    gripper(pub_setio, suction_off)
    time.sleep(1)

    # Move back up
    move_arm(pub_setjoint, Q[end_loc][3])
    time.sleep(1)

# todo: debug the start height and end height
def hanoi(n, start, mid, end,pub_setjoint, pub_setio):
    if n == 1:
        move_block(pub_setjoint, pub_setio, start, 0, end, 2)
    else:
        hanoi(n-1, start, end, mid)
        move_block(pub_setjoint, pub_setio, start, 0, end, 2)
        hanoi(n-1, mid, start, end)

 
############### Your Code End Here ###############
 
 
def main():
 
	global home
	global Q
	global SPIN_RATE

    # Definition of our tower

    # 2D layers (top view)

    # Layer (Above blocks)
    # | Q[0][2][1] Q[1][2][1] Q[2][2][1] |   Above third block
    # | Q[0][1][1] Q[1][1][1] Q[2][1][1] |   Above point of second block
    # | Q[0][0][1] Q[1][0][1] Q[2][0][1] |   Above point of bottom block

    # Layer (Gripping blocks)
    # | Q[0][2][0] Q[1][2][0] Q[2][2][0] |   Contact point of third block
    # | Q[0][1][0] Q[1][1][0] Q[2][1][0] |   Contact point of second block
    # | Q[0][0][0] Q[1][0][0] Q[2][0][0] |   Contact point of bottom block

 # TODO 
	# - problem is at the addressing of Q, which doesn't align with above 
	# - why do we need so many contact and above point? to check that, currenly, all we have is simply the contact point 
	# - if cont, can it work ? or we must to construct a same structure 
	# - 


    # First index - From left to right position A, B, C
    # Second index - From "bottom" to "top" position 1, 2, 3
    # Third index - From gripper contact point to "in the air" point

    # How the arm will move (Suggestions)
    # 1. Go to the "above (start) block" position from its base position
    # 2. Drop to the "contact (start) block" position
    # 3. Rise back to the "above (start) block" position
    # 4. Move to the destination "above (end) block" position
    # 5. Drop to the corresponding "contact (end) block" position
    # 6. Rise back to the "above (end) block" position

	# Initialize ROS node
	rospy.init_node('lab3_node')
 
    # Initialize publisher for ur3e_driver_ece470/setjoint with buffer size of 10

	pub_setjoint = rospy.Publisher('ur3e_driver_ece470/setjoint',JointTrajectory,queue_size=10)
	
	############## Your Code Start Here ##############
	# TODO: define a ROS publisher for /ur3e_driver_ece470/setio message 
	pub_setio = rospy.Publisher('/ur3e_driver_ece470/setio', Digital, queue_size=10)
 
 
	############### Your Code End Here ###############


	# Initialize subscriber to /joint_states and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('/joint_states', JointState, position_callback)
 
	############## Your Code Start Here ##############
	# TODO: define a ROS subscriber for /ur_hardware_interface/io_states message and corresponding callback function
	sub_io = rospy.Subscriber('/ur_hardware_interface/io_states', IOStates, gripper_input_callback)
 
 
	############### Your Code End Here ###############
 
 
	############## Your Code Start Here ##############
	# TODO: modify the code below so that program can get user input
 
	input_done = 0
	loop_count = 0
	start = 0
	mid = 1
	des = 2

	while(not input_done):
		input_string = input("Enter number of loops <Either 1 2 3 or 0 to quit>: ")
		print("You entered " + input_string + "\n")

		if(int(input_string) == 1):
			input_done = 1
			loop_count = 1
		elif (int(input_string) == 2):
			input_done = 1
			loop_count = 2
		elif (int(input_string) == 3):
			input_done = 1
			loop_count = 3
		elif (int(input_string) == 0):
			print("Quitting... ")
			# sys.exit()
		else:
			print("Please just enter the character 1 2 3 or 0 to quit \n\n")




			

	############### Your Code End Here ###############
 
	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")
 
	rospy.loginfo("Sending Goals ...")
 
	loop_rate = rospy.Rate(SPIN_RATE)
	# TODO what is this?
	
	############## Your Code Start Here ##############
	# TODO: modify the code so that UR3 can move tower accordingly from user input
 
	hanoi(3,start,mid, des, pub_setjoint,pub_setio)
	move_block(pub_setjoint, pub_setio, start, 0, des,   2)
	




	
 
 
	############### Your Code End Here ###############
 
 
 
if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass
