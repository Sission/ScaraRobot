#! /usr/bin/env python

import rospy
from scarabot_ik.srv import JointPose, JointPoseResponse

def callback(request):
    # let bb8 move in circle
    response = JointPoseResponse()
    response.q1 = request.x
    response.q2 = request.y
    response.d = request.z
    rospy.loginfo("q1 value is "+str(response.q1))
    return response


if __name__ == '__main__':
    rospy.init_node('inverse_kinematics', anonymous=True)
    ikservice = rospy.Service('inverse_kinematics_server', JointPose, callback)
    rospy.spin()
