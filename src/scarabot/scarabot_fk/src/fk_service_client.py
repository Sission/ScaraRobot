#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
import numpy as np
import math

l1_r = 0.62
l1_z = 1.68
l2 = 0.5
l3 = 0.0
DEBUG = True
VERBOSE = False

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
        homogenousMatrix = homogenousTransform([theta1, theta2, -d])
        self.eepose.position.x = np.float64(homogenousMatrix[0, 3])
        self.eepose.position.y = np.float64(homogenousMatrix[1, 3])
        self.eepose.position.z = np.float64(homogenousMatrix[2, 3])
        self.pub.publish(self.eepose)
        rospy.loginfo("Forward Kinematics calculated pose:")
        rospy.loginfo(self.eepose.position)
        if(VERBOSE):
            rospy.loginfo("True pose:")
            rospy.loginfo(self.trueEepose)
        if(DEBUG):
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
