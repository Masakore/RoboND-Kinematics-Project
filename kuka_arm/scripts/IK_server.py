#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        def rot_x(angle):
            return Matrix([[ 1,         0,            0],
                           [ 0, cos(angle), -sin(angle)],
                           [ 0, sin(angle),  cos(angle)]])
        def rot_y(angle):
            return Matrix([[ cos(angle),  0, sin(angle)],
                           [          0,  1,          0],
                           [-sin(angle),  0, cos(angle)]])
        def rot_z(angle):
            return Matrix([[  cos(angle), -sin(angle), 0],
                           [  sin(angle),  cos(angle), 0],
                           [      0,        0,         1]])

        def DH_transition(alpha, a, d, q):
            return Matrix([[            cos(q),           -sin(q),           0,             a],
                           [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                           [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                           [                 0,                 0,           0,             1]])

        # Initialize service response
        joint_trajectory_list = []

        # Define DH param symbols
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angle
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # joint length
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # joint offset

        # Joint angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # joint variables: theta_i
        roll, pitch, yaw = symbols('roll pitch yaw')

        # Modified DH params
        s = {alpha0:     0, a0:      0, d1:  0.75,
             alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
             alpha2:     0, a2:   1.25, d3:     0,
             alpha3: -pi/2, a3: -0.054, d4:  1.50,
             alpha4:  pi/2, a4:      0, d5:     0,
             alpha5: -pi/2, a5:      0, d6:     0,
             alpha6:     0, a6:      0, d7: 0.303, q7: 0}

        # Define Modified DH Transformation matrix

        T0_1 = DH_transition(alpha0, a0, d1, q1).subs(s)
        T1_2 = DH_transition(alpha1, a1, d2, q2).subs(s)
        T2_3 = DH_transition(alpha2, a2, d3, q3).subs(s)
        #T3_4 = DH_transition(alpha3, a3, d4, q4).subs(s)
        #T4_5 = DH_transition(alpha4, a4, d5, q5).subs(s)
        #T5_6 = DH_transition(alpha5, a5, d6, q6).subs(s)
        #T6_G = DH_transition(alpha6, a6, d7, q7).subs(s)

        R_roll = rot_x(roll)
        R_pitch = rot_y(pitch)
        R_yaw = rot_z(yaw)

        R_z = rot_z(pi)
        R_y = rot_y(-pi/2)
        R_corr = R_z * R_y

        Rrpy = R_yaw * R_pitch * R_roll * R_corr

        T0_3 = T0_1 * T1_2 * T2_3

        R0_3 = T0_3[0:3, 0:3]
        # Tried inv("LU") but gave outside of joint range
        # So I stick to normal inverse
        R3_6 = (R0_3 ** (-1)) * Rrpy

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (ee_roll, ee_pitch, ee_yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # 1. Calculate Wrist Center(Joint 5 for our case)
            P_EE = Matrix([px, py, pz])
            D = Matrix([0, 0, s[d7]])
            P_WC = P_EE - Rrpy.evalf(subs={roll: ee_roll, pitch: ee_pitch, yaw: ee_yaw}) * D
            P_WC_x, P_WC_y, P_WC_z = P_WC[0], P_WC[1], P_WC[2]

            # 2. calculate theta1
            # Simply project the coordinate of the WC into the ground plane refer to the figure 3 in RoboND-P2.md
            theta1 = atan2(P_WC_y, P_WC_x).evalf()

            # 3. calculate theta2
            # dist_J2xy_5xy: distance between the x-y-plane projections of joint2 and joint5(WC)
            # dist_j2z_5z: distance between the z-plane projections of joint2 and joint5(WC)
            # dist_J2_5: distance between joint2 and joint5
            # dist_J3_5: distance between joint3 and joint5
            # ganma: the angle consisted of dist_J2xy_5xy and dist_j2z_5z
            # beta: the angle consisted of dist_J3_5 and dist_J2_5

            dist_J2xy_5xy = sqrt(P_WC_x**2 + P_WC_y**2) - s[a1]
            dist_J2z_5z = P_WC_z - s[d1]
            dist_J2_5 = sqrt(dist_J2xy_5xy**2 + dist_J2z_5z**2)
            dist_J3_5 = sqrt(s[d4]**2 + s[a3]**2)
            ganma = atan2(dist_J2z_5z, dist_J2xy_5xy)
            cos_beta = (s[a2]**2 + dist_J2_5**2 - dist_J3_5**2)/(2*s[a2]*dist_J2_5) # Law of cos
            sin_beta = sqrt(1-cos_beta**2)
            beta = atan2(sin_beta, cos_beta)

            theta2 = (pi/2 - ganma - beta).evalf()

            # 4. calculate theta3
            # ganma2: the angle consisted of a2 and dist_J3_5
            # beta2: the angle consisted of dist_J3_5 and dist_J2_5
            cos_beta2 = (s[a2]**2 + dist_J3_5**2 - dist_J2_5**2)/(2*s[a2]*dist_J3_5)
            sin_beta2 = sqrt(1 - cos_beta2**2) # cos**2 + sin**2 = 1
            beta2 = atan2(sin_beta2, cos_beta2) # tan = sin / cos
            ganma2 = atan2(-s[a3], s[d4]) # a3 is negative all the time

            theta3 = (pi/2 - ganma2 - beta2).evalf()

            # 5. calculate theta4, 5, 6:
            R3_6_num = R3_6.evalf(subs={q1: theta1, q2: theta2, q3: theta3, roll: ee_roll, pitch: ee_pitch, yaw: ee_yaw})

            r13 = R3_6_num[0,2]
            r33 = R3_6_num[2,2]

            r21 = R3_6_num[1,0]
            r22 = R3_6_num[1,1]

            r23 = R3_6_num[1,2]

            theta5 = atan2(sqrt(1 - r23**2), r23).evalf()
            if sin(theta5) < 0:
            	theta4 = atan2(-r33, r13).evalf()
            	theta6 = atan2(r22, -r21).evalf()
            else:
            	theta4 = atan2(r33, -r13).evalf()
            	theta6 = atan2(-r22, r21).evalf()

            print ('===result of theta calculation====')
            print ('theta1: ', theta1)
            print ('theta2: ', theta2)
            print ('theta3: ', theta3)
            print ('theta4: ', theta4)
            print ('theta5: ', theta5)
            print ('theta6: ', theta6)


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
