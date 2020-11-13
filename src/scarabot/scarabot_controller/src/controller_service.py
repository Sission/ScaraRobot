#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest, ApplyJointEffort, ApplyJointEffortRequest
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import numpy as np
import math


class ControlService:
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_joint_properties')
        self.control_service = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.joint_obj = GetJointPropertiesRequest()
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        self.control_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        self.effort_obj = ApplyJointEffortRequest()
        self.reference_pose_sub = rospy.Subscriber('reference_position_pub', Float64, self.cal_effort)
        self._joint_name = "prismatic_joint"
        self._effort = 0
        self._start_time_secs = 0
        self._duration_secs = 1
        self._d = 0

    def end_effector_value(self):
        self.joint_obj.joint_name = self._joint_name
        result = self.control_service(self.joint_obj)
        self._d = result.position[0]
        rospy.loginfo("Current d values:")
        rospy.loginfo(self._d)

    def cal_effort(self, msg):
        self.end_effector_value()
        rospy.loginfo("A reference position has been received")
        reference_location = msg.data
        self._effort = 10 + self._d
        # calculate effort here

        self.effort_obj.joint_name = self._joint_name
        self.effort_obj.effort = self._effort
        self.effort_obj.start_time.secs = self._start_time_secs
        self.effort_obj.duration.secs = self._duration_secs
        result = self.control_effort(self.effort_obj)
        if result.success:
            rospy.loginfo("The effort has applied successfully")
        else:
            rospy.logdebug("The effort did not been applied")


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(10)
    control_node = ControlService()
    while not rospy.is_shutdown():
        control_node.__init__()
        rate.sleep()
