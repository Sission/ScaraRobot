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
        self._joint_name = "prismatic_joint"
        self._effort = 0
        self._ref = 0
        self._start_time_secs = 0
        self._duration_secs = 1
        self._d = 0
        self._old_t = 0
        self._old_e = 0
        self._kp = 40
        self._kd = 20

    def end_effector_value(self):
        self.joint_obj.joint_name = self._joint_name
        result = self.control_service(self.joint_obj)
        self._d = result.position[0]
        rospy.loginfo("Current d values:")
        rospy.loginfo(self._d)

    def pd_controller(self, ref):
        e = ref - self._d
        print("ref is ", ref)
        print("current d is ", self._d)
        print("e is ", e)
        t = rospy.get_time()
        print("txx is", t)
        delta_t = t - self._old_t
        e_dot = (e - self._old_e) / delta_t
        print("e_dot", e_dot)
        self._effort = self._kp * e + self._kd * e_dot
        print("duration is ", self._duration_secs)
        print("effort is ", self._effort)
        self._old_e = e
        print("e is ", self._old_e)
        self._old_t = t
        print("t is ", self._old_t)
        # self._duration_secs = delta_t*2

    def callback(self, msg):
        self._ref = msg.data
        rospy.loginfo("A reference position has been received")

    def cal_effort(self):
        self.end_effector_value()
        rospy.Subscriber('reference_position_pub', Float64, self.callback)
        reference_location = self._ref
        # print("FU", self.reference_pose_sub.callback_args)
        print("current d1 is ", self._d)
        self.pd_controller(reference_location)
        self.effort_obj.joint_name = self._joint_name
        print("effort is", self._effort)
        if abs(self._effort)< 0.01:
            self.effort_obj.effort = 0
        else:
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
    rate = rospy.Rate(60)
    control_node = ControlService()
    while not rospy.is_shutdown():
        control_node.cal_effort()
        rate.sleep()
