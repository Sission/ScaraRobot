#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest, ApplyJointEffort, ApplyJointEffortRequest
from scarabot_controller.msg import Reference
import numpy as np

DEBUG = True
VERBOSE = False


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
        self._kp = 0.5, 0.5, 2
        self._kd = 0.01, 0.01, 0.1
        self._working = [False, False, False]

    def end_effector_value(self, joint_name, i):
        self.joint_obj.joint_name = joint_name
        result = self.control_service(self.joint_obj)
        if(VERBOSE):
            print(result)
        self._d[i] = result.rate[0]

    def pd_controller(self, ref, i):
        e = ref - self._d[i]
        t = rospy.get_time()
        delta_t = t - self._old_t[i]
        e_dot = (e - self._old_e[i]) / delta_t
        self._effort[i] = self._kp[i] * e + self._kd[i] * e_dot
        self._duration_n_secs = int(min(delta_t * 1e9, 1e8))
        self._old_e[i] = e
        self._old_t[i] = t

    def callback(self, msg):
        self._ref[0] = msg.q1
        self._ref[1] = msg.q2
        self._ref[2] = msg.d
        self._working = [True, True, True]

    def cal_effort(self):
        rospy.Subscriber('all_reference_velocity_pub', Reference, self.callback)
        if(any(self._working)):
            for i in range(3):
                self.end_effector_value(self._joint_name[i], i)
                self.pd_controller(self._ref[i], i)
                self.effort_obj.joint_name = self._joint_name[i]
                if abs(self._effort[i]) < 0.01:
                    self.effort_obj.effort = 0
                    self._working[i] = False
                    if(DEBUG and not any(self._working)):
                        print("Finished")
                else:
                    self.effort_obj.effort = self._effort[i]
                    self.effort_obj.start_time.secs = self._start_time_secs
                    self.effort_obj.duration.nsecs = self._duration_n_secs
                    res = self.control_effort(self.effort_obj)


if __name__ == '__main__':
    rospy.init_node('vel_controller', anonymous=True)
    rate = rospy.Rate(100)
    control_node = ControlService()
    while not rospy.is_shutdown():
        control_node.cal_effort()
        rate.sleep()
