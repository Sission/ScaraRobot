#!/usr/bin/env python
from __future__ import print_function
import rospy
import sys
import math
import time
import genpy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from pd_controller.srv import MoveJoint, MoveJointResponse

reference_q3 = None
old_t = 0
old_e = 0
Kp = 30
Kd = 15

def callback(req):
    reference_q3 = req.data

    Q3 = req.position
    e = reference_q3 - Q3
    t = rospy.get_time()
    
    delta_t = t - old_t
    e_dot = (e - old_e)/delta_t
    U = Kp * e + Kd * e_dot
    gazebo_service = rospy.ServiceProxy(
            '/gazebo/apply_joint_effort', ApplyJointEffort)
    
    old_e = e
    old_t = t
    
    return MoveJointResponse()



if __name__ == "__main__":
    rospy.init_node('pd_controller', anonymous=True)
    ikservice = rospy.Service('move_joint3', MoveJoint, callback)
    rospy.spin()
