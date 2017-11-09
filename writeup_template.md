## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -90 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

Note that...

theta2 has a constant -90 degree rotation, reflected in the DH table

d1 = the Z delta from base to joint 2

d4 = the Z delta from joint 3 to joint 5 (WC)

d7 = the Z delta from joint 5 to end effector

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

*Insert image from DH videos*

```python
    # Create symbols for joint variables
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a1:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7 = symbols('alpha1:8')

	# Create Modified DH parameters
    s = {
        alpha0: 0,      a0: 0,      d1: 0.75,
        alpha1: -pi/2,  a1: 0.35,   d2: 0,      q2: q2 - pi/2,
        alpha2: 0,      a2: 1.25,   d3, 0,
        alpha3: -pi/2,  a3: -0.054, d4: 1.5,
        alpha4: pi/2,   a4: 0,      d5: 0,
        alpha5: -pi/2,  a5: 0,      d6: 0,
        alpha6: 0,      a6: 0,      d7: 0.303,   q7: 0,
    }

	# Define Modified DH Transformation matrix
    T0_1 = Matrix([
        [cos(q1),               -sin(q1),               0,              alpha0],
        [sin(q1)*cos(alpha0),   cos(q1)*cos(alpha0),    -sin(alpha0),   -sin(alpha0)*d1],
        [sin(q1)*cos(alpha0),   cos(q1)*cos(alpha0),    cos(alpha0),    cos(alpha0)*d1],
        [0,                     0,                      0,              1],
    ])
    T0_1 = T0_1.subs(s)


    T1_2 = Matrix([
        [cos(q2),               -sin(q2),               0,              alpha1],
        [sin(q2)*cos(alpha1),   cos(q2)*cos(alpha1),    -sin(alpha1),   -sin(alpha1)*d2],
        [sin(q2)*cos(alpha1),   cos(q2)*cos(alpha1),    cos(alpha1),    cos(alpha1)*d2],
        [0,                     0,                      0,              1],
    ])
    T1_2 = T1_2.subs(s)


    T2_3 = Matrix([
        [cos(q3),               -sin(q3),               0,              alpha2],
        [sin(q3)*cos(alpha2),   cos(q3)*cos(alpha2),    -sin(alpha2),   -sin(alpha2)*d3],
        [sin(q3)*cos(alpha2),   cos(q3)*cos(alpha2),    cos(alpha2),    cos(alpha2)*d3],
        [0,                     0,                      0,              1],
    ])
    T2_3 = T2_3.subs(s)


    T3_4 = Matrix([
        [cos(q4),               -sin(q4),               0,              alpha3],
        [sin(q4)*cos(alpha3),   cos(q4)*cos(alpha3),    -sin(alpha3),   -sin(alpha3)*d4],
        [sin(q4)*cos(alpha3),   cos(q4)*cos(alpha3),    cos(alpha3),    cos(alpha3)*d4],
        [0,                     0,                      0,              1],
    ])
    T3_4 = T3_4.subs(s)


    T4_5 = Matrix([
        [cos(q5),               -sin(q5),               0,              alpha4],
        [sin(q5)*cos(alpha4),   cos(q5)*cos(alpha4),    -sin(alpha4),   -sin(alpha4)*d5],
        [sin(q5)*cos(alpha4),   cos(q5)*cos(alpha4),    cos(alpha4),    cos(alpha4)*d5],
        [0,                     0,                      0,              1],
    ])
    T4_5 = T4_5.subs(s)


    T5_6 = Matrix([
        [cos(q6),               -sin(q6),               0,              alpha5],
        [sin(q6)*cos(alpha5),   cos(q6)*cos(alpha5),    -sin(alpha5),   -sin(alpha5)*d6],
        [sin(q6)*cos(alpha5),   cos(q6)*cos(alpha5),    cos(alpha5),    cos(alpha5)*d6],
        [0,                     0,                      0,              1],
    ])
    T5_6 = T5_6.subs(s)


    T6_EE = Matrix([
        [cos(q7),               -sin(q7),               0,              alpha6],
        [sin(q7)*cos(alpha6),   cos(q7)*cos(alpha6),    -sin(alpha6),   -sin(alpha6)*d7],
        [sin(q7)*cos(alpha6),   cos(q7)*cos(alpha6),    cos(alpha6),    cos(alpha6)*d7],
        [0,                     0,                      0,              1],
    ])
    T6_EE = T6_EE.subs(s)

	# Create individual transformation matrices
    T0_2 = simplify(T0_1 * T1_2)
    T0_3 = simplify(T0_2 * T2_3)
    T0_4 = simplify(T0_3 * T3_4)
    T0_5 = simplify(T0_4 * T4_5)
    T0_6 = simplify(T0_5 * T5_6)
    T0_EE = simplify(T0_6 * T6_EE)

```

*insert explanation for correctional rotation*

```python

    # Correct for difference in gripper link rotation between URDF and DH convention
    R_z = Matrix([
        [cos(pi),   -sin(pi),   0,  0],
        [sin(pi),   cos(pi),    0,  0],
        [0,         0,          1,  0],
        [0,         0,          0,  1]
    ])

    R_y = Matrix([
        [cos(-pi/2),    0,  sin(-pi/2), 0],
        [0,             1,  0,          0],
        [-sin(-pi/2),   0,  cos(-pi/2), 0],
        [0,         0,          0,      1]
    ])

    R_corr = simplify(R_z * R_y)
    
    # Total homogeneous transform, including correction for gripper link
    # orientation
    T_total = simplify(T0_EE * R_corr)
```



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

Let's start by grabing some of the low hanging fruit.

*insert image for calculating WC from slide 15*

theta1 = atan2(WCy, WCx), where WCy and WCx are the wrist center y and x coordinates, respectively.

We can then find the side lengths of the ABC triange defined in the image.

C = 1.25, which is the link length between joints 2 and 3 defined in the DH table.

To find B, we can use the WC coordinates, but we need to take into account the X and Z offset of joint2, as defined by a2 and d1 in the DH table.

B = sqrt( (sqrt(WCx^2 + WCy^2) - 0.35)^2 + (WCz - 0.75)^2 )

A is the length between joint3 and the wrist center (joint5).  This can be calculated as...

A = sqrt( (0.96+0.54)^2 + (-0.054)^2 ) = 1.501

Knowing A, B, and C, we can then use the Law of Cosines to determine the angle for theta2.

a = arccos( (B^2 + C^2 - A^2) / 2BC )

theta2 = pi/2 - a - atan2(WCz - d1, (sqrt(WCx^2 + WCy^2) - a1)), where d1 = 0.75 and a1 = 0.35

b = arccos( (A^2 + C^2 - B^2) / 2AC )

theta3 = pi/2 - b *ideal result includes sag for joint4*



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


