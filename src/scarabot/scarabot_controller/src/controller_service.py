#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest, ApplyJointEffort, ApplyJointEffortRequest
from geometry_msgs.msg import Pose
import numpy as np
import math

DEBUG = True
VERBOSE = False

class ControlService:
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_joint_properties')
        self.control_service = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.object_name = GetJointPropertiesRequest()
        #self.pub = rospy.Publisher('/gazebo/apply_joint_effort', Pose, queue_size=1)

    def read_joint_value(self, joint_name):
        self.object_name.joint_name = joint_name
        result = self.control_service(self.object_name)
        return result.position[0]

    def cal_end_effector_pose(self):
        theta1 = self.read_joint_value("revolute_joint_1")
        theta2 = self.read_joint_value("revolute_joint_2")
        d = self.read_joint_value("prismatic_joint")
        rospy.loginfo("Current joints:")
        rospy.loginfo(theta1)
        rospy.loginfo(theta2)
        rospy.loginfo(d)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(10)
    control_node = ControlService()
    while not rospy.is_shutdown():
        control_node.cal_end_effector_pose()
        rate.sleep()
