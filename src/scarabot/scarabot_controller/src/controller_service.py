#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest, ApplyJointEffort, ApplyJointEffortRequest
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import numpy as np
import math

DEBUG=False
VERBOSE=False

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
        self._duration_n_secs = 10000000
        self._d = 0
        self._old_t = 0
        self._old_e = 0
        self._kp = 40
        self._kd = 20

    def end_effector_value(self):
        self.joint_obj.joint_name = self._joint_name
        result = self.control_service(self.joint_obj)
        self._d = result.position[0]
        if(VERBOSE):
            rospy.loginfo("Current d values:")
            rospy.loginfo(self._d)

    def pd_controller(self, ref):
        e = ref - self._d
        if(VERBOSE):
            print("ref is ", ref)
            print("current d is ", self._d)
            print("e is ", e)
            print("txx is", t)
            print("dT is", delta_t)
            print("e_dot", e_dot)
        t = rospy.get_time()
        
        delta_t = t - self._old_t
        e_dot = (e - self._old_e) / delta_t
        self._effort = self._kp * e + self._kd * e_dot
        self._duration_n_secs = int(min((delta_t)*1e9, 1e8))
        if(DEBUG):
            print("duration is ", self._duration_n_secs)
            print("effort is ", self._effort)

        self._old_e = e
        self._old_t = t
        if(VERBOSE):
            print("e is ", self._old_e)
            print("t is ", self._old_t)

    def callback(self, msg):
        self._ref = msg.data
        if(DEBUG):
            rospy.loginfo("A reference position has been received")

    def cal_effort(self):
        self.end_effector_value()
        rospy.Subscriber('reference_position_pub', Float64, self.callback)
        reference_location = self._ref
        self.pd_controller(reference_location)
        self.effort_obj.joint_name = self._joint_name
        if(VERBOSE):
            print("current d1 is ", self._d)
            print("effort is", self._effort)
        if abs(self._effort)< 0.01:
            self.effort_obj.effort = 0
        else:
            self.effort_obj.effort = self._effort
            self.effort_obj.start_time.secs = self._start_time_secs
            self.effort_obj.duration.nsecs = self._duration_n_secs
        result = self.control_effort(self.effort_obj)
        if(result.success and DEBUG):
            rospy.loginfo("The effort has applied successfully")
        elif(DEBUG):
            rospy.logdebug("The effort did not been applied")



if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(100)
    control_node = ControlService()
    while not rospy.is_shutdown():
        control_node.cal_effort()
        rate.sleep()
