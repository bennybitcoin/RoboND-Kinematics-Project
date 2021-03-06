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
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[image4]: ./misc_images/IMG-8079.png
[image5]: ./misc_images/IMG-8080.png
[image6]: ./misc_images/IMG-8081.png
[image7]: ./misc_images/IMG-8082.png
[image8]: ./misc_images/IMG-8084.png
[image9]: ./misc_images/IMG-8085.png
[image10]: ./misc_images/IMG-8086.png
[image11]: ./misc_images/IMG-8087.png
[image12]: ./misc_images/IMG-8092.png
[image13]: ./misc_images/IMG-8093.png
[image14]: ./misc_images/pickandplace_error.PNG
[image15]: ./misc_images/results.png


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I ran the demo by following the instructions described inside the Kinematics Classroom Section. Manipulated the joints in Rviz to better understand the links of the robot arm and how the rotations affect eachother.

![alt text][image6]


I first created a sketch of the Denavit Hartenberg reference frame:

![alt text][image4]


Then I used the values from KR210 urdf to create my DH parameter table:

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054| 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

**********Additional Explanation as per First Review Request:***********

I used the <!-- joints --> section in "kuka_arm/urdf/kr210.urdf.xacro" file in order to find the distances between the origin points of the each link:

<origin xyz="0 0 0.33" rpy="0 0 0"/>
<origin xyz="0.35 0 0.42" rpy="0 0 0"/>
<origin xyz="0 0 1.25" rpy="0 0 0"/>
<origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
<origin xyz="0.54 0 0" rpy="0 0 0"/>
<origin xyz="0.193 0 0" rpy="0 0 0"/>

which tells me:

![alt text][image13]


I then used these values to derive the lengths and angles between each point via the pythagorean theorem, the meanings of the values are as follows:

alpha(i-1) : Twist angle about Z axis using RHR
a(i-1) : Link length along X axis
d(i) : Link offset along Z-axis
theta(i) : Joint angle about X-axis using RHR
q(i) : Angle of the joint (i)


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

**********Additional Mathematical as per THIRD Review Request(most recent):***********
Below are the resultant matrices for the transform from each link to the next link, I also added the file "solve.py" in the scripts file to output my total homogenous transform from the base link to the end effector
![alt text][image15]

The code below creates the full transform to the End-effector gripper pose from the fixed base link. Substituted DH parameters into the Transformation Matrix function, for each individual length between links and then multiplied them all together to find the full transformation from base to end-effector(T0_EE).

**********Additional Explanation as per First Review Request:***********

The homogenous transform is created as a combination of two rotations(R) and two translations (D) shown in the final line of equations below:

![alt text][image12]

This transformation matrix was used to create the individual transformation matrices which were then combined to create the total translation matrix from the base_link to the end_effector.

# Create Transform between base_link and gripper link (EE_rot)

    # Create Modified DH parameters
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offsets
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link lengths
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #twist angle

    # Joint angle symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i

   
    #DH Parameters
    DH = {alpha0: 0,     a0: 0,      d1: 0.75,
         alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,
         alpha2: 0,     a2: 1.25,   d3: 0,
         alpha3: -pi/2, a3: -0.054, d4: 1.50,
         alpha4: pi/2,  a4: 0,      d5: 0,
         alpha5: -pi/2, a5: 0,      d6: 0,
         alpha6: 0,     a6: 0,      d7: 0.303,  q7: 0}

    #
    # Define Modified DH Transformation matrix function
    def Trans_Matrix(alpha, a, d, q):
        TF = Matrix([[cos(q),  -sin(q),    0,  a],
            [sin(q)*cos(alpha),  cos(q)*cos(alpha),  -sin(alpha),    -sin(alpha)*d],
            [sin(q)*sin(alpha),   cos(q)*sin(alpha),  cos(alpha), cos(alpha)*d],
            [0,       0,      0,      1]])
        return TF

    #
    # Create individual transformation matrices
    T0_1 = Trans_Matrix(alpha0, a0, d1, q1).subs(DH)
    T1_2 = Trans_Matrix(alpha1, a1, d2, q2).subs(DH)
    T2_3 = Trans_Matrix(alpha2, a2, d3, q3).subs(DH)
    T3_4 = Trans_Matrix(alpha3, a3, d4, q4).subs(DH)
    T4_5 = Trans_Matrix(alpha4, a4, d5, q5).subs(DH)
    T5_6 = Trans_Matrix(alpha5, a5, d6, q6).subs(DH)
    T6_EE = Trans_Matrix(alpha6, a6, d7, q7).subs(DH)

    T0_EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)



    #IK CODE================================================
    # Extract rotation matrices from the transformation matrices
    # End effector positions:
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    #Set Roll Pitch and Yaw to end-effector postion
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    # Create Rotation Matrices

    #Variables for Rotation Matrix
    r, p, y = symbols('r p y')

    Roll_rot = Matrix([[ 1,         0,          0],
                     [ 0, cos(r), -sin(r)],
                     [ 0, sin(r), cos(r)]])

    Pitch_rot = Matrix([[ cos(p),  0, sin(p)],
                       [          0,  1,          0],
                       [-sin(p),  0, cos(p)]])

    Yaw_rot = Matrix([[ cos(y), -sin(y), 0],
                     [ sin(y),  cos(y), 0],
                     [        0,         0, 1]])

    EE_rot = Yaw_rot * Pitch_rot * Roll_rot

    #Numerically Evaluate transforms to compare with tf_echo
    # print("T0_1 = ",T0_1 = Trans_Matrix(alpha0, a0, d1, q1).subs(DH_test))
    # print("T1_2 = ",T1_2 = Trans_Matrix(alpha1, a1, d2, q2).subs(DH_test))
    # print("T2_3 = ",T2_3 = Trans_Matrix(alpha2, a2, d3, q3).subs(DH_test))
    # print("T3_4 = ",T3_4 = Trans_Matrix(alpha3, a3, d5, q4).subs(DH_test))
    # print("T4_5 = ",T4_5 = Trans_Matrix(alpha4, a4, d5, q5).subs(DH_test))
    # print("T5_6 = ",T5_6 = Trans_Matrix(alpha5, a5, d6, q6).subs(DH_test))
    # print("T6_G = ",T6_G = Trans_Matrix(alpha6, a6, d7, q7).subs(DH_test))

    # Compensate for rotation discrepancy between DH parameters and Gazebo
    # calculate error between urdf and dh
    #rotate 180 degrees around z
    R_z = Yaw_rot.subs(y, radians(180))
    #Rotate 90 degrees around y
    R_y = Pitch_rot.subs(p, radians(-90))

    R_error = simplify(R_z * R_y)

    EE_rot = EE_rot * R_error

    EE_rot = EE_rot.subs({'r': roll, 'p': pitch, 'y': yaw})


    # Wrist center calculation
    WC = Matrix([px, py, pz]) - (0.303) * EE_rot[:,2]
   

I checked that my predicted values were a good match to the actual values and applied an error correction where necessary. Ran some basic tests and was able to get the robot to grab and drop. Many times though it was knocking over the bottles and hitting into the garbage bin. I made some final adjustments(see Project Implementation) to gain a successful solution.

Submission #3:

I corrected my IK calculations for my joint angles (see next section). Therefore I could resume using the normal Wrist Center position without any error adjustments.



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Theta1 was simply calcualted as the rotation for the wrist center position. I reduced this by 12% in order to account for my lengthining of the wrist center distance DH[d7].

Theta 2 and 3 were a little more difficult to calculate. I created a triangle using the postions of links 2,3,4. The triangle sides are labeled side_a,b,c and the interior angles were calculated via the Cosine law and labeled angle_a,b,c. I then derived theta 2 and 3 using the interior angles I had just solved for. (see image below)


![alt text][image10]

Theta 4,5,6 required me to solve the following equation and extract Euler angles from rotation matrixes:

![alt text][image11]


# Calculate joint angles using Geometric IK method

    theta1 = atan2(WC[1],WC[0])
    WC_average = WC[0] * WC[0] + WC[1] * WC[1]
    # SSS triangle
    side_a = 1.501
    side_b = sqrt(pow((sqrt(WC_average) - DH[a1]), 2) + pow((WC[2] - DH[d1]), 2))
    side_c = 1.25

    angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
    angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
    #angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))
    angle_d = atan2(WC[2] - DH[d1], sqrt(WC_average) - DH[a1])

    theta2 = pi/2 - angle_a - angle_d ### removed my adjustment from previous adjustment
    theta3 = pi/2 - (angle_b + 0.036) #-0.054m sag in link 4
    Rot_03 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    Rot_03 = Rot_03.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    Rot_36 = Rot_03.inv("LU") * EE_rot

    #Euler Angles from Rotation Matrix
    theta4 = atan2(Rot_36[2,2], -Rot_36[0,2])
    theta5 = atan2(sqrt(Rot_36[0,2]*Rot_36[0,2] + Rot_36[2,2]*Rot_36[2,2]),Rot_36[1,2])
    theta6 = atan2(-Rot_36[1,1], Rot_36[1,0])


    For this most recent submission I recalculated my angle formulas, and was able to achieve a much lower for my thetas <0.0026. See image below:




### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

## Submission 2:
In my initial runs my robot arm seemed to go in too far each time, therby knocking over the cans. In order to compensate for this error I made the decision to extend my DH value for d7 from 0.303 to 0.36663(a 21% distance). By moving the wrist center position out like this, I caused the robot to use the far edge of its gripper for grasping. This caused it to have less issues with knocking the cans.

The final issue I had to correct was the robot arm not accurately achieveing the center of the trash bin. (see image below) I reduced my theta1 by 12% and theta2 by 31% and I was able to achieve a position in the center of the bin to drop. I believe this error was an effect of me artificially increasing the wrist center length to solve the previous issue

## Current Submission #3:

After changing my IK formulas for my theta angles, I was able to resume using the normal 0.303 value for my wrist center position. I went through and corrected my parameters to match what I derived in part one and removed any hardcoded constant values I was using for error adjustments as per the comments on my last submission. 


successful 8/10 run(below). 
![alt text][image9]
