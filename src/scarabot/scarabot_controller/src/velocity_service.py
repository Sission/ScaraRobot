#!/usr/bin/env python
import math
import numpy as np
import rospy
from scarabot_controller.srv import GetEndEffectorVelocity, GetEndEffectorVelocityResponse, GetJointVelocity, GetJointVelocityResponse
from std_msgs.msg import Float64
import csv
import os


DEBUG = True
VERBOSE = False


def eeVelocityToJointV(eV):
    # robot body values
    l1_r = 0.62
    l1_z = 1.68
    l2 = 0.5
    l3 = 0.0

    # joint angles
    q1 = math.pi / 2
    q2 = math.pi/2
    q3 = 0

    # end-effector velocities
    eeVelMatrix = np.array([eV.vx,
                            eV.vy,
                            eV.vz,
                            eV.wx,
                            eV.wy,
                            eV.wz])

    jacobian = np.array([
        [-(l2 * s(q1) * c(q2) + l2 * c(q1) * s(q2) + l1_r * s(q1)), - (l2 * s(q1) * c(q2) + l2 * c(q1) * s(q2)), 0],
        [l2 * c(q1) * c(q2) - l2 * s(q1) * s(q2) + l1_r * c(q1), l2 * c(q1) * c(q2) - l2 * s(q1) * s(q2), 0],
        [0, 0, 1],
        [0, 0, 0],
        [0, 0, 0],
        [1, 1, 0]])

    jtrans = np.transpose(jacobian)
    j_plus = np.matmul(np.linalg.inv(np.matmul(jtrans, jacobian)), jtrans)


    jointVel = np.matmul(j_plus, eeVelMatrix)

    if(DEBUG):
        print("Joint velocities: " + np.array2string(jointVel))
    return jointVel


def jointToEndEffector(jointVelocities):
    # robot body values
    l1_r = 0.62
    l1_z = 1.68
    l2 = 0.5
    l3 = 0.0

    # joint angles
    q1 = math.pi/2
    q2 = math.pi/2
    q3 = 0

    # joint velocities
    [q1dot, q2dot, q3dot] = [jointVelocities.q1, jointVelocities.q2, jointVelocities.q3]
    jointmat = np.array([[q1dot],
                         [q2dot],
                         [q3dot]])

    jacobian = np.array([
        [-(l2 * s(q1) * c(q2) + l2 * c(q1) * s(q2) + l1_r * s(q1)), - (l2 * s(q1) * c(q2) + l2 * c(q1) * s(q2)), 0],
        [l2 * c(q1) * c(q2) - l2 * s(q1) * s(q2) + l1_r * c(q1), l2 * c(q1) * c(q2) - l2 * s(q1) * s(q2), 0],
        [0, 0, 1],
        [0, 0, 0],
        [0, 0, 0],
        [1, 1, 0]])

    ee = np.matmul(jacobian, jointmat)
    if DEBUG:
        print("End effector velocities: " + np.array2string(ee))
    return ee


def c(value):
    return math.cos(value)


def s(value):
    return math.sin(value)


def main():
    jvelocities = [3.5, 2, 1]
    ee = jointToEndEffector(jvelocities)
    eeVelocityToJointV(ee)

    eeVels = [-2.17,-2.75,1,0,0,5.5]
    jVels = eeVelocityToJointV(eeVels)
    jointToEndEffector(jVels)

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