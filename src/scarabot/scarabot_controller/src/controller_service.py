#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest, ApplyJointEffort, ApplyJointEffortRequest
from geometry_msgs.msg import Pose
import numpy as np
import math
from scarabot_controller.msg import JointEffortRequest

DEBUG = True
VERBOSE = False

class ControlService:
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_joint_properties')
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        self.monitoring_service = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.object_name = GetJointPropertiesRequest()
        self.control_service = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        self.target_name = ApplyJointEffortRequest()
        self.sub = rospy.Subscriber("scarabot_controller_requests", JointEffortRequest, self.request_joint_effort_callback, queue_size=1)
        #self.pub = rospy.Publisher('/gazebo/apply_joint_effort', Pose, queue_size=1)

    def read_joint_value(self, joint_name):
        self.object_name.joint_name = joint_name
        result = self.monitoring_service(self.object_name)
        return result.position[0]

    def request_joint_effort_callback(self, request):
        self.target_name.joint_name = request.joint_name
        self.target_name.effort = request.effort
        self.target_name.duration = request.duration
        result = self.control_service(self.target_name)

    def log_joint_values(self):
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
        control_node.log_joint_values()
        rate.sleep()
