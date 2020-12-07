#! /usr/bin/env python

import rospy
from scarabot_ik.srv import JointPose, JointPoseResponse

from geometry_msgs.msg import Pose
import numpy as np
import math


def ik_calcu(x, y, z):
    cosq2 = round((np.square(x) + np.square(y) - 0.6344) * 50 / 31, 6)
    q2 = np.arctan2(np.sqrt(1 - np.square(cosq2)), cosq2)
    # , np.sqrt(1 - np.square(w))
    q3 = z - 1.68
    cosq1 = ((cosq2 / 2 + 0.62) * x + np.sin(q2) / 2 * y) / (np.square(cosq2 / 2 + 0.62) + np.square(np.sin(q2) / 2))
    q1 = np.arctan2(np.sqrt(1 - np.square(cosq1)), cosq1)
    return q1, q2, q3


def callback(request):
    response = JointPoseResponse()
    range = round((np.square(request.x) + np.square(request.y) - 0.6344) * 50 / 31, 6)
    if -1 <= range <= 1:
        q1, q2, d = ik_calcu(request.x, request.y, request.z)
        response.q1 = q1
        response.q2 = q2
        response.d = d
        rospy.loginfo("theta_1 value is " + str(response.q1))
        rospy.loginfo("theta_2 value is " + str(response.q2))
        rospy.loginfo("d value is " + str(response.d))
    else:
        rospy.loginfo("Pose of end effector is out of the workspace")
    return response



if __name__ == '__main__':
    rospy.init_node('inverse_kinematics', anonymous=True)
    ikservice = rospy.Service('inverse_kinematics_server', JointPose, callback)
    rospy.spin()
