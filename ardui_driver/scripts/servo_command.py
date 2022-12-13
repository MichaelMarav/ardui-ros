#! /usr/bin/env python3 

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from ecattie_msgs.msg import GoalStates
from ecattie_msgs.msg import JointStates

if __name__ == "__main__":
    rospy.init_node('simple_node',anonymous=True)
    # msg_n = GoalStates()
    # msg_n.front_left.goal_angle.append(5)

    # pub = rospy.Publisher("test_topic",GoalStates,queue_size=1)
    # # msg = JointState()
    rate = rospy.Rate(50)
    # i = 0
    # while not rospy.is_shutdown():
    #     pub.publish(msg_n)
    #     rate.sleep()
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("joint_states",JointStates)
        print(msg.front_left.angle)
        rate.sleep()