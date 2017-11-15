#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
        	joint_trajectory_point = JointTrajectoryPoint()
 
		# Create Modified DH parameters
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

    #
    # Extract rotation matrices from the transformation matrices
    # End effector positions:
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z
    
    # Wrist center calculation
    WC = Matrix([px, py, pz]) - (DH[d7]*1.21) * EE_rot[:,2]


    # Calculate joint angles using Geometric IK method
    theta1 = atan2(WC[1],WC[0])
    WC_average = WC[0] * WC[0] + WC[1] * WC[1]
    # SSS triangle
    side_a = 1.501
    side_b = sqrt(pow((sqrt(WC_average) - 0.35), 2) + pow((WC[2] - 0.75), 2))
    side_c = 1.25

    angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
    angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))    
    angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

    theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC_average) - 0.35)
    theta3 = pi / 2 - (angle_b + 0.036) #-0.054m sag in link 4
    Rot_03 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    Rot_03 = Rot_03.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    Rot_36 = Rot_03.inv("LU") * EE_rot

    #Euler Angles from Rotation Matrix
    theta4 = atan2(Rot_36[2,2], -Rot_36[0,2])
    theta5 = atan2(sqrt(Rot_36[0,2]*Rot_36[0,2] + Rot_36[2,2]*Rot_36[2,2]),Rot_36[1,2])
    theta6 = atan2(-Rot_36[1,1], Rot_36[1,0])

        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
    joint_trajectory_list.append(joint_trajectory_point)

    rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
