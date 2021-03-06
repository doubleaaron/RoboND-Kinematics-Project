## Project: Robotic Arm Kinematics Pick & Place

---

[//]: # (Image References)

[image1]: ./misc_images/launch_kuka_urdf.jpg
[image2]: ./misc_images/denavit_hartenberg_sketch.jpg
[image3]: ./misc_images/wc_equation.jpg
[image4]: ./misc_images/euler_equation.jpg
[image5]: ./misc_images/wc_homogeneous_matrix.jpg
[image6]: ./misc_images/calculate_three_thetas_01.jpg
[image7]: ./misc_images/calculate_three_thetas.jpg
[image8]: ./misc_images/three_thetas_xy.jpg
[image9]: ./misc_images/kuka_arm_9_out_of_10.JPG

---

### Project Writeup / Kuka KR210 Pick and Place

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To run Forward Kinematics I switched to ~/catkin_ws directory, sourced devel/setup.bash and ran roslaunch forward_kinematics.launch

Launched /catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/urdf/kr210.urdf.xacro to perform kinematic analysis and derive Denavit Hartenberg parameters (JJ Craig method).

![alt text][image1]

![alt text][image2]

The instructions for DH parameter assignment process for open kinematic chains with n degrees of freedom (i.e., joints) is summarized as:

1. Label all joints from {1, 2, … , n}.
2. Label all links from {0, 1, …, n} starting with the fixed base link as 0.
3. Draw lines through all joints, defining the joint axes.
4. Assign the Z-axis of each frame to point along its joint axis.
5. Identify the common normal between each frame ​Z​^​i−1 and frame Z​^​i.

6. The endpoints of "intermediate links" (i.e., not the base link or the end effector) are associated with two joint axes, {i} and {i+1}. For i from 1 to n-1, assign the X^​i
to be...

    a. For skew axes, along the normal between Z^​i and Z​^​i+1 and pointing from {i} to {i+1}.
    
    b. For intersecting axes, normal to the plane containing ​Z^​i and Z^​i+1.
    
    c. For parallel or coincident axes, the assignment is arbitrary; look for ways to make other DH parameters equal to zero.

7. For the base link, always choose frame {0} to be coincident with frame {1} when the first joint variable (θ​1 or d1) is equal to zero. This will guarantee that α0 = a​0 = 0, and, if joint 1 is a revolute, d​1 = 0. If joint 1 is prismatic, then θ​1 = 0.

8. For the end effector frame, if joint n is revolute, choose Xn to be in the direction of X​n−1 when θ​n = 0 and the origin of frame {n} such that d​n = 0

## DH Parameter Table:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0
  
Parameter | Name | Definition   
--- | --- | ---
alpha(i-1) | Twist angle | Angle from z(i-1) axis to z(i) axis measured along the x(i-1) axis
a(i-1) | Link length | Distance from z(i-1) axis to z(i) axis measured along the x(i-1) axis
d(i) | Link offset | Distance from x(i-1) axis to x(i) axis measured along the z(i) axis
theta(i) | Joint variable | Angle from x(i-1) axis to x(i) axis measured along the z(i) axis
  
>>> Alpha(i-1) = Twist Angle

>>> a(i-1) = Distance

>>> d(i) = Link length

>>> theta(i) (Oi in diagram) for joint angles.

>>> In joint 2, there is an offset of -90 degrees between x(1) and x(2).

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

```
T0-T1
[cos(q₁)  -sin(q₁)  0   0]
[sin(q₁)  cos(q₁)   0   0]
[   0        0      1  0.75]
[   0        0      0   1]

T1-T2
cos(q₂ - 0.5π)   -sin(q₂ - 0.5/π)  0  0.35
       0                 0          1   0  
-sin(q₂ - 0.5π)  -cos(q₂ - 0.5/π)  0   0  
       0                 0          0   1  

T2-T3
cos(q₃)  -sin(q₃)  0  1.25
sin(q₃)  cos(q₃)   0   0  
   0        0      1   0  
   0        0      0   1  

T3-T4
cos(q₄)   -sin(q₄)  0  -0.054
   0         0      1   1.5  
-sin(q₄)  -cos(q₄)  0    0   
   0         0      0    1   

T4-T5
cos(q₅)  -sin(q₅)  0   0
   0        0      -1  0
sin(q₅)  cos(q₅)   0   0
   0        0      0   1

T5-T6
cos(q₆)   -sin(q₆)  0  0
   0         0      1  0
-sin(q₆)  -cos(q₆)  0  0
   0         0      0  1

T6 or Gripper
1  0  0    0  
0  1  0    0  
0  0  1  0.303
0  0  0    1

T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```
#### Python, Sympy, Defining Functions, etc.
```python
# Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #joint variables, thetas
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offsets
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link lengths
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, = symbols('alpha:7') #twist angles
    

# Create Modified DH parameters
	dh_table = {alpha0:     0, a0:      0, d1:  0.75, q1:        q1,
                    alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2 - pi/2,
                    alpha2:     0, a2:   1.25, d3:     0, q3:        q3,
                    alpha3: -pi/2, a3: -0.054, d4:   1.5, q4:        q4,
                    alpha4:  pi/2, a4:      0, d5:     0, q5:        q5,
                    alpha5: -pi/2, a5:      0, d6:     0, q6:        q6,
                    alpha6:     0, a6:      0, d7: 0.303, q7:         0}
```

```python
# Put Transformation Matrix into a function

# Define Modified DH Transformation matrix
	def create_TM(alpha, a, d, q):
    	T = Matrix([[            cos(q),           -sin(q),           0,             a],   
                [ cos(alpha)*sin(q), cos(alpha)*cos(q), -sin(alpha), -sin(alpha)*d],
                [ sin(alpha)*sin(q), sin(alpha)*cos(q),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
    	return T

# Create individual transformation matrices
	

    T0_1 = create_TM(alpha0, a0, d1, q1).subs(dh_table)
    T1_2 = create_TM(alpha1, a1, d2, q2).subs(dh_table)
    T2_3 = create_TM(alpha2, a2, d3, q3).subs(dh_table)
    T3_4 = create_TM(alpha3, a3, d4, q4).subs(dh_table)
    T4_5 = create_TM(alpha4, a4, d5, q5).subs(dh_table)
    T5_6 = create_TM(alpha5, a5, d6, q6).subs(dh_table)
    T6_G = create_TM(alpha6, a6, d7, q7).subs(dh_table)


    T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```
OR
```
T0_3 = T0_1 * T1_2 * T2_3
T0_G = T0_3 * T3_4 * T4_5 * T5_6 * T6_G
```

### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

#### Step 1: Complete the DH Table. Completed earlier.

#### Step 2: Find the location of the WC relative to the base frame.

![alt text][image5]:

![alt text][image3]

#### Step 3: Find joint angles(variables) q1, q2 and q3, such that the WC has coordinates equal to previous equation.

Symbolically define our homogeneous transform:
```
[lx  mx  nx  px]
[ly  my  ny  py]
[lz  mz  nz  pz]
[0   0   0   1]
```
Where l, m and n are orthonormal vectors representing the end-effector orientation along X, Y, Z axes of the local coordinate frame.

Since n is the vector along the z-axis of the gripper_link, we can say the following:
```
wcx = px - (d6 + l) * nx
wcy = py - (d6 + l) * ny
wcz = pz - (d6 + l) * nz
```
Where,

Px, Py, Pz = end-effector positions

Wx, Wy, Wz = wrist positions

d6 = from DH table

l = end-effector length

To calculate nx, ny and nz, rotation matrices are created with corrections for the difference between the URDF and the DH reference frames for the end-effector.

```
	    R_x = Matrix([[   1,      0,       0],
                          [   0, cos(r), -sin(r)],
                          [   0, sin(r),  cos(r)]]) #YAW

            R_y = Matrix([[ cos(p),  0, sin(p)],
                          [      0,  1,      0],
                          [-sin(p),  0, cos(p)]]) #PITCH

            R_z = Matrix([[ cos(y), -sin(y),  0],
                          [ sin(y),  cos(y),  0],
                          [      0,       0,  1]]) #ROLL

            R_G = R_z * R_y * R_x
            R_corr = R_z.subs(y, pi) * R_y.subs(p, -pi/2)
            R_G = R_G * R_corr
```

Extract wrist position:

nx = Rrpy[0,2]

ny = Rrpy[1,2]

nz = Rrpy[2,2]


(from transformation.euler_from_quaternion)

```
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
```


#### Step 4: Once the first three joint variables are known, calculate ​0​3​​R via application of homogeneous transforms up to the WC.
![alt text][image6]:

![alt text][image7]:

![alt text][image8]:


#### Step 5: Find a set of Euler angles corresponding to the rotation matrix.

Where:
```
R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6

and

R0_6 = Rrpy
```
Rrpy = Homogeneous RPY rotation between base_link and gripper_link as calculated above.

We can substitute the values we calculated for joints 1 to 3 in their respective individual rotation matrices and pre-multiply both sides of the above equation by inv(R0_3) which leads to:

R3_6 = inv(R0_3) * Rrpy

Precalculate Thetas 1,2,3. While calculating the inverse above, using Sympy's inv() method, please make sure to pass "LU" as an argument.

```
R0_3 = T0_3[:3,:3]

R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

R3_6 = R0_3.inv('LU') * R_G
```

Theta 4,5,6 are derived from R3_6 using Euler Angles from Rotation Matrix.

```
theta4 = atan2(r36[2, 2], -r36[0, 2])
theta5 = atan2(sqrt(r36[0, 2]^2 + r36[2, 2]^2), r36[1, 2])
theta6 = atan2(-r36[1, 1], r36[1, 0])
```

CODE:
```
gamma = atan2(WC[2] - d1, sqrt(WC[0]**2 + WC[1]**2) - a1)
beta = atan2(d4, -a3)

theta1 = atan2(WC[1], WC[0])
theta2 = pi/2 - angle_A - gamma
theta3 = -(angle_B - beta)

theta4 = atan2(-R[2,2], R[0,2])
theta5 = atan2(sqrt(R[0,2]**2 + R[2,2]**2), R[1,2])
theta6 = atan2(R[1,1], -R[1,0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


My first two simulations in Gazebo+RViz didn't go so well. The arm came into the picking area way too fast and the gripper didn't open prior to entry in to the zone, resulting in cylinders being knocked over in nearly every attempt. Debugging revealed some silly indents causing variable not to initialize. From Common Questions section I inserted the following code in line 327 in the /src/trajectory_sampler.cpp file:

```ros::Duration(2.0).sleep();
```
I then I added R3_6 = R0_3.inv('LU') * R_G from the notes.

Occasionally I would get alot of IK solves to the drop off container that wouldn't follow through from RViz to Gazebo that I couldn't figure out besides clicking the Next button too quickly. I gave some time from that point on and it went fairly smoothly from there on out.

Changed (from notes):
```
#From this
R3_6 = R0_3.inv('LU') * R_G

#To this
R3_6 = R0_3.transpose() * R_G
```
And it sped things up a great deal.

I tried out some pickle methods to gain some more speed.

Attibutions:

I used the Video Walkthrough whenever I got stuck in the lectures, and it was extremely helpful for getting the symbolic vs code issues I was having sorted out. Harsh's code really helped me link up the concept of Symbolic Code. I feel like I could easily forget all of this if I didn't do it every day, so will need to refresh it quite a bit going into the future. It's alot of information and concepts. Maybe a game engine programmer feels like they are good at this?

In the future I would try out numpy instead of sympy for speed. Maybe even taking it to the point of using numba within numpy and CUDA with decorators. I've had some amazing increases in computation speed using that method for deep learning, maybe it could work outside of the VM on an Ubuntu Workstation with nvidia gpus. That would be a super fun experiment in the future.

#### Voila! Finally got one with a 9/10 after a couple hours and quite a few failed sessions.

![alt text][image9]


