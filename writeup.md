## Project: Robotic Arm Kinematics Pick & Place

---

[//]: # (Image References)

[image1]: ./misc_images/launch_kuka_urdf.jpg
[image2]: ./misc_images/denavit_hartenberg_sketch.jpg
[image3]: ./misc_images/wc_equation.png

---

### Project Writeup / Kuka KR210 Pick and Place

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To run Forward Kinematics I switched to ~/catkin_ws directory, sourced devel/setup.bash and ran roslaunch forward_kinematics.launch

Launched /catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/urdf/kr210.urdf.xacro to perform kinematic analysis and derive Denavit Hartenberg parameters (JJ Craig method).

![alt text][image1]

![alt text][image2]

DH Parameter Table:

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

8. For the end effector frame, if joint n is revolute, choose Xn to be in the direction of X​n−1 when θ​n = 0 and the origin of frame {n} such that d​n = 0.



```python
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


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

![alt text][image3]

And here's where you can draw out and show your math for the derivation of your theta angles. 



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


