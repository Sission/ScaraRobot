#!/usr/bin/env python
import math
import numpy as np
import rospy
from scarabot_controller.srv import GetEndEffectorVelocity, GetEndEffectorVelocityResponse, GetJointVelocity, \
    GetJointVelocityResponse
from std_msgs.msg import Float64
import csv
import os

DEBUG = True
VERBOSE = False


def tranformationmatrix(t, d, a, al):
    trans = np.array([[math.cos(t), -math.sin(t) * math.cos(al), math.sin(t) * math.sin(al), a * math.cos(t)],
                      [math.sin(t), math.cos(t) * math.cos(al), -math.cos(t) * math.sin(al), a * math.sin(t)],
                      [0, math.sin(al), math.cos(al), d],
                      [0, 0, 0, 1]])
    return trans


def jaco_matrix(t1, t2, d):
    a1 = np.matmul(tranformationmatrix(0, 0.6, 0, 0), tranformationmatrix(t1, 0.83, 0.62, 0))
    a2 = np.matmul(a1, tranformationmatrix(t2, 0.25, 0.5, 0))
    a3 = np.matmul(a2, tranformationmatrix(0, d, 0, 0))
    z0 = np.array([0, 0, 1])
    z1 = a1[0:3, 2]
    z2 = a2[0:3, 2]
    o0 = np.array([0, 0, 0])
    o1 = a1[0:3, 3]
    o2 = a2[0:3, 3]
    o3 = a3[0:3, 3]
    jv1 = np.cross(z0, (o3 - o0))
    jv2 = np.cross(z1, (o3 - o1))
    jv3 = z2
    jw1 = z0
    jw2 = z1
    jw3 = np.array([0, 0, 0])
    jacobian = np.zeros((6, 3))
    jacobian[0:3, 0] = jv1
    jacobian[0:3, 1] = jv2
    jacobian[0:3, 2] = jv3
    jacobian[3:6, 0] = jw1
    jacobian[3:6, 1] = jw2
    jacobian[3:6, 2] = jw3
    return jacobian


def eeVelocityToJointV(eV):
    # joint angles
    q1 = math.pi / 2
    q2 = math.pi / 2
    q3 = 0

    # end-effector velocities
    eeVelMatrix = np.array([eV.vx,
                            eV.vy,
                            eV.vz,
                            eV.wx,
                            eV.wy,
                            eV.wz])

    jacobian = jaco_matrix(q1, q2, q3)
    jtrans = np.transpose(jacobian)
    j_plus = np.matmul(np.linalg.inv(np.matmul(jtrans, jacobian)), jtrans)

    jointVel = np.matmul(j_plus, eeVelMatrix)

    if (DEBUG):
        print("Joint velocities: " + np.array2string(jointVel))
    return jointVel


def jointToEndEffector(jointVelocities):
    # joint angles
    q1 = math.pi / 2
    q2 = math.pi / 2
    q3 = 0

    # joint velocities
    [q1dot, q2dot, q3dot] = [jointVelocities.q1, jointVelocities.q2, jointVelocities.q3]
    jointmat = np.array([[q1dot],
                         [q2dot],
                         [q3dot]])

    jacobian = jaco_matrix(q1, q2, q3)

    ee = np.matmul(jacobian, jointmat)
    if DEBUG:
        print("End effector velocities: " + np.array2string(ee))
    return ee


def c(value):
    return math.cos(value)


def s(value):
    return math.sin(value)


def j2eCallback(jV):
    ret = GetEndEffectorVelocityResponse()
    ee = jointToEndEffector(jV)
    ret.vx = ee[0]
    ret.vy = ee[1]
    ret.vz = ee[2]
    ret.wx = ee[3]
    ret.wy = ee[4]
    ret.wz = ee[5]
    return ret


def e2jCallback(ee):
    ret = GetJointVelocityResponse()
    jV = eeVelocityToJointV(ee)
    ret.q1 = jV[0]
    ret.q2 = jV[1]
    ret.q3 = jV[2]
    return ret


def initVelocityServer():
    rospy.init_node('velocity_service', anonymous=True)
    rate = rospy.Rate(100)
    s1 = rospy.Service('Joint2EEVelocity', GetEndEffectorVelocity, j2eCallback)
    s2 = rospy.Service('EE2JointVelocity', GetJointVelocity, e2jCallback)
    rospy.spin()


if __name__ == '__main__':
    initVelocityServer()
