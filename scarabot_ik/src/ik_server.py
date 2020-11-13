#!/usr/bin/env python
from __future__ import print_function
import rospy
import sys
import math
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Quaternion
from scarabot_ik.srv import JointPose, JointPoseResponse

def callback(req):

    print("The joint 1 pose is ")
    print(req.x)
    print("The joint 2 pose is ")
    print(req.y)
    print("The joint 3 (prismatic) pose is ")
    print(req.z)

    X = req.x
    Y = req.y
    Z = req.z

    cos_q2 = (X**2+Y**2-0.62**2-0.5**2)/(2*0.62*0.5)
    q2 = -math.acos(cos_q2)    
    q1 = math.atan2(Y, X)-math.atan2(0.5*math.sin(q2),0.62+0.5*math.cos(q2))
    q3 = Z  

    response = JointPoseResponse()
    response.q1 = q1
    response.q2 = q2
    response.q3 = q3
    rospy.loginfo("q1 value is "+str(response.q1))
    return response



if __name__ == "__main__":
    rospy.init_node('inverse_kinematics', anonymous=True)
    ikservice = rospy.Service('inverse_kinematics_server', JointPose, callback)
    rospy.spin()
