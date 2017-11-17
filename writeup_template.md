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

I ran the demo by following the instructions described inside the Kinematics Classroom Section. Manipulated the joints in Rviz to better understand the links of the robot arm and how the rotations affect eachother.

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.



Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054| 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0



# Create Transform between base_link and gripper link (EE_rot)
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offsets
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link lengths
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #twist angle

    # Joint angle symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

    #Variables for Rotation Matrix
    r, p, y = symbols('r p y')
    #x, y, z = symbols('x y z')

    DH = {alpha0: 0,     a0: 0,      d1: 0.75,
         alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,
         alpha2: 0,     a2: 1.25,   d3: 0,
         alpha3: -pi/2, a3: -0.054, d4: 1.50,
         alpha4: pi/2,  a4: 0,      d5: 0,
         alpha5: -pi/2, a5: 0,      d6: 0,
         alpha6: 0,     a6: 0,      d7: 0.303, q7: 0}
    #            
    # Define Modified DH Transformation matrix
    def Trans_Matrix(alpha, a, d, q):
    	TF = Matrix([[				cos(q),		-sin(q),	0,		a],
    		[	sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
    		[	sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha), cos(alpha)*d],
    		[ 					0,					0,			0,				1]])
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

    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

    #Set Roll Pitch and Yaw to end-effector postion
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    # Create Rotation Matrices
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

    # Compensate for rotation discrepancy between DH parameters and Gazebo
    # calculate error between urdf and dh
    #rotate 180 degrees around z
    R_z = Yaw_rot.subs(y, radians(180))
    #Rotate 90 degrees around y
    R_y = Pitch_rot.subs(p, radians(-90))

    R_error = simplify(R_z * R_y)

    EE_rot = EE_rot * R_error

    EE_rot = EE_rot.subs({'r': roll, 'p': pitch, 'y': yaw})




#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


In my initial runs my robot arm seemed to go in too far each time, therby knocking over the cans. In order to compensate for this error I made the decision to extend my DH value for d7 from 0.303 to 0.36663(a 21% distance). By moving the wrist center position out like this, I caused the robot to use the far edge of its gripper for grasping. This caused it to have less issues with knocking the cans.


And just for fun, another example image:
![alt text][image3]


