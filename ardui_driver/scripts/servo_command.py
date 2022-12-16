#! /usr/bin/env python3 


'''
    Script that publishes the final desired angles of the servo motors in degrees 
'''

import rospy
from ecattie_msgs.msg import GoalStates



# Set the desired angles and vels
desired_angles=[ 90 ,120,150]
desired_vels  = [30,30,30]


if __name__ == "__main__":
    rospy.init_node('simple_node',anonymous=True)
    rate = rospy.Rate(1)

    goal_pub = rospy.Publisher("/servo/servo_goal",GoalStates,queue_size=1)
    goal_msg = GoalStates()

    for i in range(len(desired_angles)):
        goal_msg.goal_angles.append(desired_angles[i])
        goal_msg.goal_vels.append(desired_vels[i])



    while not rospy.is_shutdown():
        goal_pub.publish(goal_msg)
        rate.sleep()