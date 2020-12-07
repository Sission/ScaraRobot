#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest, ApplyJointEffort, ApplyJointEffortRequest
from scarabot_controller.msg import Reference
import numpy as np

DEBUG = False
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
        self._old_t = 0
        self._old_e = 0
        self._kp = 40, 40, 40
        self._kd = 20, 20, 20

    def end_effector_value(self, joint_name, i):
        self.joint_obj.joint_name = joint_name
        result = self.control_service(self.joint_obj)
        self._d[i] = result.rate[0]
        if VERBOSE:
            rospy.loginfo("Current d values:")
            rospy.loginfo(self._d)

    def pd_controller(self, ref, i):
        e = ref - self._d[i]

        t = rospy.get_time()

        delta_t = t - self._old_t
        e_dot = (e - self._old_e) / delta_t
        self._effort[i] = self._kp[i] * e + self._kd[i] * e_dot
        self._duration_n_secs = int(min(delta_t * 1e9, 1e8))
        if VERBOSE:
            print("ref is ", ref)
            print("current d is ", self._d)
            print("e is ", e)
            print("txx is", t)
            print("dT is", delta_t)
            print("e_dot", e_dot)
            print("duration is ", self._duration_n_secs)
            print("effort is ", self._effort)

        self._old_e = e
        self._old_t = t
        if VERBOSE:
            print("e is ", self._old_e)
            print("t is ", self._old_t)

    def callback(self, msg):
        self._ref[0] = msg.q1
        self._ref[1] = msg.q2
        self._ref[2] = msg.d
        if DEBUG:
            rospy.loginfo("A reference position has been received")

    def cal_effort(self):
        rospy.Subscriber('all_reference_velocity_pub', Reference, self.callback)
        for i in range(3):
            self.end_effector_value(self._joint_name[i], i)
            reference_location = self._ref[i]
            self.pd_controller(reference_location, i)
            self.effort_obj.joint_name = self._joint_name[i]
            if abs(self._effort[i]) < 0.01:
                self.effort_obj.effort = 0
            else:
                self.effort_obj.effort = self._effort[i]
                self.effort_obj.start_time.secs = self._start_time_secs
                self.effort_obj.duration.nsecs = self._duration_n_secs
            res = self.control_effort(self.effort_obj)
        # prismatic_location = self._ref[1]
        # self.pd_controller(prismatic_location)
        # self.effort_obj.joint_name = self._joint_name1
        # if VERBOSE:
        #     print("current d1 is ", self._d)
        #     print("effort is", self._effort)
        # if abs(self._effort) < 0.01:
        #     self.effort_obj.effort = 0
        # else:
        #     self.effort_obj.effort = self._effort
        #     self.effort_obj.start_time.secs = self._start_time_secs
        #     self.effort_obj.duration.nsecs = self._duration_n_secs
        # result = self.control_effort(self.effort_obj)
        # with open(os.getcwd()+"/data.csv", "a") as csvfile:
        #     writer = csv.writer(csvfile)
        #     writer.writerow([rospy.get_time(), self._d, reference_location])
        # if result.success and DEBUG:
        #     rospy.loginfo("The effort has applied successfully")
        # elif DEBUG:
        #     rospy.logdebug("The effort did not been applied")


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(100)
    # csvfile = open(os.getcwd()+"/data.csv", "w")
    # writer = csv.writer(csvfile)
    # writer.writerow(["time", "current position", "reference position"])
    # csvfile.close()
    control_node = ControlService()
    while not rospy.is_shutdown():
        control_node.cal_effort()
        rate.sleep()
