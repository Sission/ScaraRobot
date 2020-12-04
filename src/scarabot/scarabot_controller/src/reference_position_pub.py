#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('reference', Float64, queue_size=10)
    rospy.init_node('reference_position_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    ref = Float64()
    ref.data = 10
    while not rospy.is_shutdown():
        rospy.loginfo("Reference position data has been published")
        pub.publish(ref)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass