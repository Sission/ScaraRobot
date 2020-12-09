#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Pose
import numpy as np
import math
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest, ApplyJointEffort, ApplyJointEffortRequest
from scarabot_controller.msg import Reference
from scarabot_controller.srv import GetEndEffectorVelocity, GetEndEffectorVelocityResponse, GetJointVelocity, \
    GetJointVelocityResponse
from std_msgs.msg import Float64


class YMoving:
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_joint_properties')
        self.control_service = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.joint_obj = GetJointPropertiesRequest()
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        self.control_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        self.effort_obj = ApplyJointEffortRequest()
        self._joint_name = "revolute_joint_1", "revolute_joint_2", "prismatic_joint"
        self._effort = np.zeros(3)
        self._veffort = np.zeros(3)
        self._ref = np.zeros(3)
        self._start_time_secs = 0
        self._duration_n_secs = 10000000
        self._d = np.zeros(3)
        self._v = np.zeros(3)
        self._old_t = np.zeros(3)
        self._old_e = np.zeros(3)
        self._oldv_t = np.zeros(3)
        self._oldv_e = np.zeros(3)
        self._kp = 100, 100, 40
        self._kd = 40, 40, 20
        self._vkp = 0.5, 0.5, 1
        self._vkd = 0.01, 0.01, 0.5
        self.initial_pose = self.ik_calcu(0, 0.2, 1.68)
        self.y_direction_vel = 0

    def ik_calcu(self, x, y, z):
        cosq2 = round((np.square(x) + np.square(y) - 0.6344) * 50 / 31, 6)
        q2 = np.arctan2(np.sqrt(1 - np.square(cosq2)), cosq2)
        q3 = z - 1.68
        cosq1 = ((cosq2 / 2 + 0.62) * x + np.sin(q2) / 2 * y) / (
                np.square(cosq2 / 2 + 0.62) + np.square(np.sin(q2) / 2))
        q1 = np.arctan2(np.sqrt(1 - np.square(cosq1)), cosq1)
        return q1, q2, q3

    def tranformationmatrix(self, t, d, a, al):
        trans = np.array([[math.cos(t), -math.sin(t) * math.cos(al), math.sin(t) * math.sin(al), a * math.cos(t)],
                          [math.sin(t), math.cos(t) * math.cos(al), -math.cos(t) * math.sin(al), a * math.sin(t)],
                          [0, math.sin(al), math.cos(al), d],
                          [0, 0, 0, 1]])
        return trans

    def jaco_matrix(self, t1, t2, d):
        a1 = np.matmul(self.tranformationmatrix(0, 0.6, 0, 0), self.tranformationmatrix(t1, 0.83, 0.62, 0))
        a2 = np.matmul(a1, self.tranformationmatrix(t2, 0.25, 0.5, 0))
        a3 = np.matmul(a2, self.tranformationmatrix(0, d, 0, 0))
        z0 = np.array([0, 0, 1])
        z1 = a1[0:3, 2]
        z2 = a2[0:3, 2]
        o0 = np.array([0, 0, 0])
        o1 = a1[0:3, 3]
        o2 = a2[0:3, 3]
        o3 = a3[0:3, 3]
        jv1 = np.cross(z0, (o3 - o0))
        jv2 = np.cross(z1, (o3 - o1))
        jv3 = z2
        jw1 = z0
        jw2 = z1
        jw3 = np.array([0, 0, 0])
        jacobian = np.zeros((6, 3))
        jacobian[0:3, 0] = jv1
        jacobian[0:3, 1] = jv2
        jacobian[0:3, 2] = jv3
        jacobian[3:6, 0] = jw1
        jacobian[3:6, 1] = jw2
        jacobian[3:6, 2] = jw3
        return jacobian

    def eeVelocityToJointV(self, eV):
        eeVelMatrix = eV
        jacobian = self.jaco_matrix(self._d[0], self._d[1], self._d[2])
        jtrans = np.transpose(jacobian)
        j_plus = np.matmul(np.linalg.inv(np.matmul(jtrans, jacobian)), jtrans)
        jointVel = np.matmul(j_plus, eeVelMatrix)
        return jointVel

    def c(self, value):
        return math.cos(value)

    def s(self, value):
        return math.sin(value)

    def end_effector_value(self, joint_name, i):
        self.joint_obj.joint_name = joint_name
        result = self.control_service(self.joint_obj)
        self._d[i] = result.position[0]
        self._v[i] = result.rate[0]
    #
    # def read_joint_value(self, joint_name, i):
    #     self.joint_obj.joint_name = joint_name
    #     result = self.control_service(self.joint_obj)


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

    def vel_pd_controller(self, ref, i):
        e = ref - self._v[i]
        t = rospy.get_time()
        delta_t = t - self._oldv_t[i]
        if delta_t == 0:
            delta_t = 0.00001
        e_dot = (e - self._oldv_e[i]) / delta_t
        self._veffort[i] = self._vkp[i] * e + self._vkd[i] * e_dot
        self._vduration_n_secs = int(min(delta_t * 1e9, 1e8))
        self._oldv_e[i] = e
        self._oldv_t[i] = t

    def callback(self, msg):
        self.y_direction_vel = msg.data

    def cal_effort(self):
        rospy.Subscriber('y_direction_vel', Float64, self.callback)
        velocity = np.array([0, self.y_direction_vel, 0, 0, 0, 0])

        if self.y_direction_vel == 0:
            for i in range(3):
                self.end_effector_value(self._joint_name[i], i)
                self.pd_controller(self.initial_pose[i], i)
                self.effort_obj.joint_name = self._joint_name[i]

                if abs(self._effort[i]) < 0.01:
                    self.effort_obj.effort = 0
                else:
                    self.effort_obj.effort = self._effort[i]
                    self.effort_obj.start_time.secs = self._start_time_secs
                    self.effort_obj.duration.nsecs = self._duration_n_secs
                res = self.control_effort(self.effort_obj)
        else:
            for i in range(2):
                self.end_effector_value(self._joint_name[i], i)
                # print "read end effort value now is", self._d[0], self._d[1]
                print velocity
                joint_velocity = self.eeVelocityToJointV(velocity)
                print "calculated joint velocity now is", joint_velocity[0], joint_velocity[1]
                print "read joint velocity now is", self._v[0], self._v[1]

                # self.read_joint_value(self._joint_name[i], i)

                self.vel_pd_controller(joint_velocity[i], i)
                self.effort_obj.joint_name = self._joint_name[i]
                if abs(self._veffort[i]) < 0.01:
                    self.effort_obj.effort = 0
                else:
                    self.effort_obj.effort = self._veffort[i]
                    self.effort_obj.start_time.secs = self._start_time_secs
                    self.effort_obj.duration.nsecs = self._vduration_n_secs
                res = self.control_effort(self.effort_obj)

            self.end_effector_value(self._joint_name[2], 2)
            self.pd_controller(self.initial_pose[2], 2)
            self.effort_obj.joint_name = self._joint_name[2]
            self.effort_obj.effort = self._effort[2]
            self.effort_obj.start_time.secs = self._start_time_secs
            self.effort_obj.duration.nsecs = self._duration_n_secs
            res = self.control_effort(self.effort_obj)



if __name__ == '__main__':
    rospy.init_node('y_trajectory', anonymous=True)
    rate = rospy.Rate(20)
    control_node = YMoving()
    while not rospy.is_shutdown():
        control_node.cal_effort()
        rate.sleep()
