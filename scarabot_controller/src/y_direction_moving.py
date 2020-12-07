#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import numpy as np
import math
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest, ApplyJointEffort, ApplyJointEffortRequest
from scarabot_controller.msg import Reference

def ik_calcu(x, y, z):
    cosq2 = round((np.square(x) + np.square(y) - 0.6344) * 50 / 31, 6)
    q2 = np.arctan2(np.sqrt(1 - np.square(cosq2)), cosq2)
    q3 = z - 1.68
    cosq1 = ((cosq2 / 2 + 0.62) * x + np.sin(q2) / 2 * y) / (np.square(cosq2 / 2 + 0.62) + np.square(np.sin(q2) / 2))
    q1 = np.arctan2(np.sqrt(1 - np.square(cosq1)), cosq1)
    return q1, q2, q3


# initial position
initial_pose = ik_calcu(0, 0.2, 1.68)


class ControlService:
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_joint_properties')
        self.control_service = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.joint_obj = GetJointPropertiesRequest()
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        self.control_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        self.effort_obj = ApplyJointEffortRequest()
        self._joint_name = "revolute_joint_1", "revolute_joint_2", "prismatic_joint"
        self._effort = np.zeros(3)
        self._ref = np.zeros(3)
        self._start_time_secs = 0
        self._duration_n_secs = 10000000
        self._d = np.zeros(3)
        self._old_t = np.zeros(3)
        self._old_e = np.zeros(3)
        self._kp = 100, 100, 40
        self._kd = 40, 40, 20

    def end_effector_value(self, joint_name, i):
        self.joint_obj.joint_name = joint_name
        result = self.control_service(self.joint_obj)
        self._d[i] = result.position[0]

    def pd_controller(self, ref, i):

        e = ref - self._d[i]
        t = rospy.get_time()
        delta_t = t - self._old_t[i]
        if delta_t == 0:
            delta_t = 0.00001
        e_dot = (e - self._old_e[i]) / delta_t
        self._effort[i] = self._kp[i] * e + self._kd[i] * e_dot
        self._duration_n_secs = int(min(delta_t * 1e9, 1e8))
        self._old_e[i] = e
        self._old_t[i] = t


    def cal_effort(self):
        for i in range(3):
            self.end_effector_value(self._joint_name[i], i)
            self.pd_controller(initial_pose[i], i)
            self.effort_obj.joint_name = self._joint_name[i]
            if abs(self._effort[i]) < 0.01:
                self.effort_obj.effort = 0
            else:
                self.effort_obj.effort = self._effort[i]
                self.effort_obj.start_time.secs = self._start_time_secs
                self.effort_obj.duration.nsecs = self._duration_n_secs
            res = self.control_effort(self.effort_obj)


if __name__ == '__main__':
    rospy.init_node('y_trajectory', anonymous=True)
    rate = rospy.Rate(100)
    control_node = ControlService()
    while not rospy.is_shutdown():
        control_node.cal_effort()
        rate.sleep()
