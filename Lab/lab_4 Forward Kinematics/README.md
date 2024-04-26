# README

## 1. Task

Calculate the forward kinematics of UR3e using PoE method.

- base frame: robot's base frame 
- target frame: suction cup's frame shown in the manual
- compute transformation from target frame to base frame

### what to do 

1. decide your initial state and compute matrix `M`

   any pose is fine theoretically. what to do is to minus bias of joint angles.

   The manual will recommend one pose for you.

2. according to PoE method, decide and compute all things.

## 2. Code

- `lab4_exec.py`: main code to execute forward kinematics
- `lab4_func.py`: function to compute forward kinematics using PoE method (*TODO*)
- `test_fk.cpp`: verify code (*NO MODIFICTION*)

## 3. utils

### 3.1 how to run code

```bash
$ python lab4_exec.py theta1 theta2 theta3 theta4 theta5 theta6
```

### 3.2 how to verify answer offline

```bash
$ roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.1.120
# (new terminal)
$ python lab4_exec.py theta1 theta2 theta3 theta4 theta5 theta6
# (new terminal) build this code before run it
$ rosrun lab4pkg_fk test_fk
```

### 3.3 How to verify answer online

```bash
$ python lab4_exec.py 90.00 0.00 0.00 -90.00 0.00 0.00
# Forward kinematics calculated with gripper: 
# 3.55283e-06 0.00565484 0.999984 0.29008
# 0.000628308 0.999984 -0.00565484 -0.543749
# -1 0.000628318 -2.05104e-10 0.151796
# 0 0 0 1

$ python lab4_exec.py 59.59 -45.26 -27.50 3.44 -13.18 -31.51
# Forward kinematics calculated with gripper: 
# -0.263007 0.395416 0.880042 0.0955197
# -0.0699828 0.901933 -0.426167 -0.407005
# -0.962252 -0.173673 -0.209543 0.463785
# 0 0 0 1

$ python lab4_exec.py 93.39 -71.62 -9.74 -88.24 -30.94 48.13
# Forward kinematics calculated with gripper: 
# -0.318115 0.343549 0.883616 0.28087
# -0.446133 0.76814 -0.459267 -0.191219
# -0.836522 -0.54031 -0.0910886 0.662953
# 0 0 0 1
```

### 3.4 How to connect your robot

```bash
$ roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.1.120

# create a program with External Control program in Teach Pendant and start program after launching the driver.
# when you see following responses in the terminal, that means Robot is ready!
$ [INFO] [1571124040.693851608]: Robot requested program
$ [INFO] [1571124040.693924407]: Sent program to robot
$ [INFO] [1571124040.772090597]: Robot ready to receive control commands.
```

### 3.5 How to compile your workspace

```bash
$ cd catkin_(yourID)
$ catkin_make
```

### 3.6 How to source your workspace (before open a new terminal )

```bash
$ cd catkin_(yourID)
$ source devel/setup.bash
```

### 3.7 How to make your script executable

```bash
$ cd catkin_(yourID)/src/lab4pkg_fk/scripts 
$ chmod +x lab4_exec.py
```







# LAB 4 Report Template

## Introduction

This lab focuses on implementing the exponential forward kinematics for the UR3e robot to estimate the end-effector pose given a set of joint angles. The lab aims to bridge the theoretical aspects of robot kinematics with practical application using ROS and Python.

## Objectives

- Solve the exponential forward kinematic equations for the UR3e robot.
- Implement a Python function to move the UR3e robot to a configuration specified by the user.
- Compare the exponential forward kinematic estimation with the actual robot movement.

## Theoretical Solution

### Overview

The forward kinematic equations were derived for the UR3e robot using the exponential forward kinematics method. The transformation matrix \(T_{06}\) was solved, detailing the matrices defined by \(e^{[S_1]\theta_1}\) through \(e^{[S_6]\theta_6}\).

### Screw Axes and Transformation Matrix

Screw axes \(S_i\) and the zero configuration transformation matrix \(M\) for the UR3e robot were determined as follows:

- **Screw Axes (S1 to S6):**

| Joint | \(S_i = (\omega, v)\) |
| ----- | --------------------- |
| S1    | (Fill in)             |
| S2    | (Fill in)             |
| ...   | ...                   |
| S6    | (Fill in)             |

- **Transformation Matrix (M):** (Describe M matrix here)

### Calculated Matrices

The exponential matrices for each joint movement \(e^{[S_i]\theta_i}\) are as follows:

- \(e^{[S_1]\theta_1}\) = (Fill in)
- ...
- \(e^{[S_6]\theta_6}\) = (Fill in)

## Physical Implementation

A Python function was written to move the UR3e to specified joint angles and calculate the corresponding end-effector pose. The joint angles provided by the user are in degrees and adhere to the robot's joint limits.

### Implementation Details

- ROS Package: `lab4pkg_fk`
- Main Files: `lab4_exec.py`, `lab4_func.py`

### Execution Command

```bash
rosrun lab4pkg_fk lab4_exec.py [theta1] [theta2] [theta3] [theta4] [theta5] [theta6]
