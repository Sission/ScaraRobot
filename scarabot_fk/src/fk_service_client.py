#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
import numpy as np
import math

DEBUG = True
VERBOSE = False

def tranformationmatrix(t, d, a, al):
    trans = np.array([[math.cos(t), -math.sin(t) * math.cos(al), math.sin(t) * math.sin(al), a * math.cos(t)],
                      [math.sin(t), math.cos(t) * math.cos(al), -math.cos(t) * math.sin(al), a * math.sin(t)],
                      [0, math.sin(al), math.cos(al), d],
                      [0, 0, 0, 1]])
    return trans


class FkService:
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_joint_properties')
        self.fk_service = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.object_name = GetJointPropertiesRequest()
        self.pub = rospy.Publisher('forward_kinematics_INFO', Pose, queue_size=1)
        self.subscriber = rospy.Subscriber("gazebo/link_states", LinkStates, self.updateCallback, queue_size=1)
        self.eepose = Pose()
        self.trueEepose = Pose()

    def read_joint_value(self, joint_name):
        self.object_name.joint_name = joint_name
        result = self.fk_service(self.object_name)
        return result.position[0]

    def cal_end_effector_pose(self):
        theta1 = self.read_joint_value("revolute_joint_1")
        theta2 = self.read_joint_value("revolute_joint_2")
        d = self.read_joint_value("prismatic_joint")
        trans1 = tranformationmatrix(0, 0.6, 0, 0)
        trans2 = tranformationmatrix(theta1, 0.83, 0.62, 0)
        trans3 = tranformationmatrix(theta2, 0.25, 0.5, 0)
        trans4 = tranformationmatrix(0, d, 0, 0)
        A1 = np.matmul(trans1, trans2)
        A12 = np.matmul(A1, trans3)
        H = np.matmul(A12, trans4)
        # homogenousMatrix = homogenousTransform([theta1, theta2, d])
        self.eepose.position.x = np.float64(H[0, 3])
        self.eepose.position.y = np.float64(H[1, 3])
        self.eepose.position.z = np.float64(H[2, 3])
        # self.eepose.position.x = np.float64(homogenousMatrix[0, 3])
        # self.eepose.position.y = np.float64(homogenousMatrix[1, 3])
        # self.eepose.position.z = np.float64(homogenousMatrix[2, 3])
        self.pub.publish(self.eepose)
        rospy.loginfo("Forward Kinematics calculated")
        # rospy.loginfo(self.eepose.position)
        # if (VERBOSE):
        #     rospy.loginfo("True pose:")
        #     rospy.loginfo(self.trueEepose)
        if (DEBUG):
            self.printError()

    def printError(self):
        try:
            error = Pose()
            error.position.x = self.eepose.position.x - self.trueEepose.x
            error.position.y = self.eepose.position.y - self.trueEepose.y
            error.position.z = self.eepose.position.z - self.trueEepose.z
            rospy.loginfo("Pose error:")
            rospy.loginfo(error.position)
        except Exception as e:
            rospy.logdebug("Pose not initialized")

    def updateCallback(self, data):
        self.trueEepose = data.pose[4].position


if __name__ == '__main__':
    rospy.init_node('foward_kinematics', anonymous=True)
    rate = rospy.Rate(10)
    forward_kinematics = FkService()
    while not rospy.is_shutdown():
        forward_kinematics.cal_end_effector_pose()
        rate.sleep()
