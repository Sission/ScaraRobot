#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from std_msgs.msg import String


def callback(data):
    rospy.loginfo("The theta value of joint 1 is")
    rospy.loginfo("The theta value of joint 2 is")
    rospy.loginfo("The d value of the primistic joint is")
    rospy.loginfo("Calculated end effector pose")
    rospy.loginfo(rospy.get_caller_id() + " end effector pose has been pulished")
    eefpose = Pose()
    eefpose.position.x = data.pose[4].position.x
    eefpose.position.y = data.pose[4].position.y
    eefpose.position.z = data.pose[4].position.z
    pub = rospy.Publisher('fk_info', Pose, queue_size=10)
    pub.publish(eefpose)


def listener():
    rospy.init_node('fk_sub', anonymous=True)

    rospy.Subscriber("gazebo/link_states", LinkStates, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
