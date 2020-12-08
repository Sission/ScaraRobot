#! /usr/bin/env python


import rospy
import numpy as np
from scarabot_controller.srv import GetEndEffectorVelocity, GetEndEffectorVelocityResponse, GetJointVelocity, \
    GetJointVelocityResponse
from scarabot_controller.msg import Reference

DEBUG = True
VERBOSE = False


if __name__ == '__main__':
    rospy.init_node('p4_controller', anonymous=True)
    rate = rospy.Rate(100)
    pub0 = rospy.Publisher('all_reference_position_pub', Reference, queue_size=1)
    for i in range(300):
    	pub0.publish(1,1,0.3)
    	rate.sleep()
    rospy.Rate(0.25).sleep()
    if(DEBUG):
    	print("Second homing")
    for i in range(300):
    	pub0.publish(1,1,0.3)
    	rate.sleep()
    rospy.Rate(1).sleep()
    rospy.Rate(1).sleep()
    if(DEBUG):
    	print("Switching to velocity controls")
    rospy.wait_for_service('/EE2JointVelocity')
    joint_calculator = rospy.ServiceProxy('/EE2JointVelocity', GetJointVelocity)
    pub = rospy.Publisher('/all_reference_velocity_pub', Reference, queue_size=1)
    ref = Reference()
    targetY = 0.05
    while not rospy.is_shutdown():
        targets = joint_calculator(0,targetY,0,0,0,0)
        ref.q1 = targets.q1
        ref.q2 = targets.q2
        ref.d = targets.q3
        pub.publish(ref)
        rate.sleep()
