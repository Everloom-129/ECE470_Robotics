# ECE470 HW1



**Part a) How many independent variables would be required to fully describe the position and pose of the:**

I. **Aerial robot in free 3D space?**
An aerial robot (the four-rotor drone) in 3D space has **6** degrees of freedom:

- 3 for positioning (x, y, and z coordinates)
- 3 for orientation (roll, pitch, and yaw angles)

This is because the drone can move forward/backward, left/right, up/down, and can rotate around its longitudinal axis (roll), lateral axis (pitch), and vertical axis (yaw).

II. **Mobile ground robot, if the wheels are constrained to be always in contact with the ground?**
A ground robot, if it's a standard vehicle with non-steerable wheels, has **3** degrees of freedom:

- 2 for positioning (x and y coordinates on the ground plane)
- 1 for orientation (yaw angle)

We assume that the ground robot cannot move vertically and it cannot roll or pitch significantly.

**Part b) If the robot arm has six joints, what is the dimension of the configuration space?**

**The dimension of the configuration space is 6.**

The configuration space dimension is equal to the number of independent joints for a robot arm, as each independent joint represents one DOF. Each joint angle represents one independent variable in the configuration space.

Here, we assumed that:

- each joint provides one DOF 
- there are no kinematic constraints that couple the movements of the joints.



To fill in the Denavit-Hartenberg (D-H) parameters for the robot shown in Figure 2, we first need to understand the D-H convention, which uses four parameters to define the relationship between adjacent links in a robotic manipulator:

1. $ \alpha_{i-1} $: the twist angle between $ Z_{i-1} $ and $ Z_i $, measured about $ X_{i-1} $.
2. $ a_{i-1} $: the link length between $ Z_{i-1} $ and $ Z_i $, measured along $ X_{i-1} $.
3. $ \theta_i $: the joint angle between $ X_{i-1} $ and $ X_i $, measured about $ Z_i $.
4. $ d_i $: the link offset between $ X_{i-1} $ and $ X_i $, measured along $ Z_i $.

**a) Fill in the missing information of the axes and D-H parameters from (i)-(viii):**

From the figure:

(i) $ \alpha_0 $ is the angle between $ Z_0 $ and $ Z_1 $, which looks orthogonal. Thus, $ \alpha_0 = 90^{\circ} $.

(ii) $ d_1 $ is the offset along $ Z_1 $ from $ X_0 $ to $ X_1 $, which is $ q_1 $ because it's a prismatic joint.

(iii) $ a_1 $, the link length between $ Z_1 $ and $ Z_2 $, seems to be zero because $ X_1 $ and $ X_2 $ coincide.

(iv) $ \alpha_1 $ is the angle between $ Z_1 $ and $ Z_2 $, which also appears to be orthogonal. Thus, $ \alpha_1 = 90^{\circ} $.

(v) $ d_2 $ is the offset along $ Z_2 $ from $ X_1 $ to $ X_2 $, which is $ q_2 $ because it's a prismatic joint.

(vi) $ a_2 $, the link length between $ Z_2 $ and $ Z_3 $, seems to be zero because $ X_2 $ and $ X_3 $ coincide.

(vii) $ \alpha_2 $ is the angle between $ Z_2 $ and $ Z_3 $, which is in line or parallel. Thus, $ \alpha_2 = 0^{\circ} $.

(viii) $ d_3 $ is the offset along $ Z_3 $ from $ X_2 $ to $ X_3 $, which is zero because it is a revolute joint.

So the parameters are:
(i) $ \alpha_0 = 90^{\circ} $
(ii) $ d_1 = q_1 $
(iii) $ a_1 = 0 $
(iv) $ \alpha_1 = 90^{\circ} $
(v) $ d_2 = q_2 $
(vi) $ a_2 = 0 $
(vii) $ \alpha_2 = 0^{\circ} $
(viii) $ d_3 = 0 $

**b) Obtain the position of the center of mass $ m $, $ {}^3p $ in terms of $ q_1, q_2, $ and $ q_3 $.**

Since we're looking for $ {}^3p $, the position of the center of mass in the frame of the third joint, and all the joints are prismatic except the last one, which is revolute, the position will be only affected by $ q_1 $ and $ q_2 $. The revolute joint $ q_3 $ will only affect the orientation of the end-effector but not the position of the mass center in $ {}^3p $.

Therefore, $ {}^3p $ is:
$$ {}^3p = \begin{bmatrix} 0 \\ q_2 \\ q_1 \end{bmatrix} $$





**c) If the desired position of the center of mass $ m $, $ {}^3p $ is:**
$$ {}^3p = \begin{bmatrix} \frac{1}{\sqrt{2}} \\ \frac{1}{\sqrt{2}} \\ 1 \end{bmatrix} $$

To find the joint coordinates of the robot manipulator that would result in this position, we need to solve for $ q_1 $, $ q_2 $, and $ q_3 $.

From the previous analysis, we concluded that $ {}^3p $ in terms of $ q_1 $ and $ q_2 $ (since $ q_3 $ doesn't affect the position for this setup) is:

$$ {}^3p = \begin{bmatrix} 0 \\ q_2 \\ q_1 \end{bmatrix} $$

We can equate this to the desired position to solve for $ q_1 $ and $ q_2 $:

$$ \begin{bmatrix} 0 \\ q_2 \\ q_1 \end{bmatrix} = \begin{bmatrix} \frac{1}{\sqrt{2}} \\ \frac{1}{\sqrt{2}} \\ 1 \end{bmatrix} $$

From the equations, we can see that:

$ q_2 = \frac{1}{\sqrt{2}} $

$ q_1 = 1 $

$ q_3 $ is not determined by the position vector $ {}^3p $, as it is a revolute joint which affects orientation rather than position in this configuration. To fully define the pose of the end-effector, we would need additional information about the desired orientation. However, for the scope of this question which only concerns the position of the center of mass, $ q_3 $ can be any value. Therefore, the joint coordinates that satisfy the given position $ {}^3p $ are:

$ q_1 = 1 $ (meters or units of length)

$ q_2 = \frac{1}{\sqrt{2}} $ (meters or units of length)

$ q_3 $ can be any value (radians or degrees, depending on the unit system used for the angle).

So the answer is:

$ q_1 = 1 $, $ q_2 = \frac{1}{\sqrt{2}} $, and $ q_3 $ is indeterminate from the given information.