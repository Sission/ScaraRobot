#!/usr/bin/env python
from __future__ import print_function
import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Quaternion
from scarabot_ik.srv import joint_pose,joint_poseResponse

def cal_joint_pose(req):
    print("The joint 1 pose is ")
    print(req.x)
    print("The joint 2 pose is ")
    print(req.y)
    print("The joint 3 (prismatic) pose is ")
    print(req.z)
    return joint_poseResponse(req.x, req.y, req.z, req.x+1, req.y+1, req.z+1)



def ik_server():
    rospy.init_node('ik_node')
    rospy.Service('ik_service', joint_pose, cal_joint_pose)
    w = print("PLease input desired end effector pose")
    rospy.spin()


if __name__ == "__main__":
    ik_server()
