#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest, ApplyJointEffort, ApplyJointEffortRequest
from std_msgs.msg import Float64
import csv
import os


def callback_jointstates(msg):

    global joint1_pos, joint2_pos, joint3_pos
    joint1_pos = msg.position[0]
    joint2_pos = msg.position[1]
    joint3_pos = msg.position[2]

if __name__ == '__main__':
    # initializations

    rospy.init_node('controller', anonymous=True)

    end_effector_value()
    rospy.Subscriber('reference_position_pub', Float64, callback_jointstates)
    reference_location = _ref
    pd_controller(reference_location)
    effort_obj.joint_name = _joint_name


    joint1_pos_pub = rospy.Publisher('/scara/joint1_position_controller', Float64)
    joint2_pos_pub = rospy.Publisher('/scara/joint2_position_controller', Float64)
    joint3_pos_pub = rospy.Publisher('/scara/joint3_position_controller', Float64)


    rospy.wait_for_service('/gazebo/get_joint_properties')
    control_service = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    joint_obj = GetJointPropertiesRequest()
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    control_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
    effort_obj = ApplyJointEffortRequest()

    # Start Position Controllers
    req = GetJointPropertiesRequest()
    req.start_controllers = ["joint1_position_controller",
                             "joint2_position_controller",
                             "joint3_position_controller",
                             ]
    
    ## Unload if previously loaded, Only used when same code was running without completing
    req.stop_controllers = ["joint1_velocity_controller",
                             "joint2_velocity_controller",
                             "joint3_velocity_controller",
                             ]

    response = controller_service(req)

    initial_position = [0.3, 1.8, -0.02]  ## Move robot to a specific point

    ## Move robot to a specific point
    joint1_pos_pub.publish(initial_position[0])
    joint2_pos_pub.publish(initial_position[1])
    joint3_pos_pub.publish(initial_position[2])

    rospy.sleep()
