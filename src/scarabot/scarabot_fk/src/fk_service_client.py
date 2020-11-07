#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest
from geometry_msgs.msg import Pose
import numpy as np


class FkService:
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_joint_properties')
        self.fk_service = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.object_name = GetJointPropertiesRequest()
        self.pub = rospy.Publisher('forward_kinematics_INFO', Pose, queue_size=1)
        self.eepose = Pose()

    def read_joint_value(self, joint_name):
        self.object_name.joint_name = joint_name
        result = self.fk_service(self.object_name)
        return result.position

    def cal_end_effector_pose(self):
        theta1 = self.read_joint_value("revolute_joint_1")
        theta2 = self.read_joint_value("revolute_joint_2")
        d = self.read_joint_value("joint_prismatic")
        self.eepose.position.x = np.float64(theta1)
        self.eepose.position.y = np.float64(theta2)
        self.eepose.position.z = np.float64(d)
        self.pub.publish(self.eepose)
        rospy.loginfo("The End Effector pose has been published")


if __name__ == '__main__':
    rospy.init_node('foward_kinematics', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        forward_kinematics = FkService()
        forward_kinematics.cal_end_effector_pose()
        rate.sleep()
