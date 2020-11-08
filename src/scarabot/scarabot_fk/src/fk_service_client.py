#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest
from geometry_msgs.msg import Pose
import numpy as np
import math

l1_r = 0.615
l1_z = 1.135
l2 = 0.4
l3 = 0.3
z_start = 0.835

def homogenousTransform(jointValues):
    [q1, q2, q3] = jointValues
    ret = np.array([[1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
    revolute_link1_transform = np.array([
        [math.cos(q1),-math.sin(q1),0,l1_r*math.cos(q1)],
        [math.sin(q1), math.cos(q1),0,l1_r*math.sin(q1)],
        [0,0,1,l1_z],
        [0,0,0,1]])
    ret = np.matmul(ret, revolute_link1_transform)
    revolute_link2_transform = np.array([
        [math.cos(q2), math.sin(q2),0,l2*math.cos(q2)],
        [math.sin(q2), -math.cos(q2),0,l2*math.sin(q2)],
        [0,0,-1,0],
        [0,0,0,1]])
    ret = np.matmul(ret, revolute_link2_transform)
    prismatic_link_transform = np.array([
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,l3+q3],
        [0,0,0,1]])
    ret = np.matmul(ret, prismatic_link_transform)
    return ret

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
        return result.position[0]

    def cal_end_effector_pose(self):
        theta1 = self.read_joint_value("revolute_joint_1")
        theta2 = self.read_joint_value("revolute_joint_2")
        d = self.read_joint_value("prismatic_joint")
        homogenousMatrix = homogenousTransform([theta1, theta2, d])
        self.eepose.position.x = np.float64(homogenousMatrix[0, 3])
        self.eepose.position.y = np.float64(homogenousMatrix[1, 3])
        self.eepose.position.z = np.float64(homogenousMatrix[2, 3])
        self.pub.publish(self.eepose)
        rospy.loginfo(self.eepose)


if __name__ == '__main__':
    rospy.init_node('foward_kinematics', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        forward_kinematics = FkService()
        forward_kinematics.cal_end_effector_pose()
        rate.sleep()
