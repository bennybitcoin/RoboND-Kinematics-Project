# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

 # Create Modified DH parameters
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link lengths
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #twist angle

# Joint angle symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i

###Kuka KR210###
#DH Parameters
DH = {alpha0: 0,     a0: 0,      d1: 0.75,
     alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,
     alpha2: 0,     a2: 1.25,   d3: 0,
     alpha3: -pi/2, a3: -0.054, d4: 1.50,
     alpha4: pi/2,  a4: 0,      d5: 0,
     alpha5: -pi/2, a5: 0,      d6: 0,
     alpha6: 0,     a6: 0,      d7: 0.303,  q7: 0}
#DH test parameters
# DH_test = {alpha0: 0,     a0: 0,      d1: 0.75,  q1:0,
#       alpha1: 0, a1: 0.35,   d2: 0,  q2: q2-pi/2,
#       alpha2: 0,     a2: 1.25,   d3: 0,  q3: 0,
#       alpha3: -pi/2, a3: -0.054, d4: 1.50,   q4: 0,
#       alpha4: pi/2,  a4: 0,      d5: 0,  q5: 0,
#       alpha5: -pi/2, a5: 0,      d6: 0,  q6: 0,
#       alpha6: 0,     a6: 0,      d7: 0.303,  q7: 0}

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

print(T0_EE)