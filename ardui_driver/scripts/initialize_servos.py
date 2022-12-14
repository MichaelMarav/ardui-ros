#! /usr/bin/env python3
import rospy
from ardui_msgs.msg import GoalStates
import numpy as np



NUM_SERVOS = 3



if __name__ == "__main__":
    rospy.init_node('Initializer')
    msg = GoalStates()
    publish_topic_name = rospy.get_param("/servo_goal_topic")

    r = rospy.Rate(1) # 1hz


    pub = rospy.Publisher(publish_topic_name, GoalStates, queue_size=10)

    servos = np.arange(NUM_SERVOS)
    
    servo_param_names = []
    angvel_param_names = []
    prefix_servo = "servo"
    prefix_vel = "w"
    
    for i in servos:
        servo_param_names.append(prefix_servo + str(i))
        angvel_param_names.append(prefix_vel + str(i))


    servo_angles = np.arange(NUM_SERVOS)
    servo_vels   = np.arange(NUM_SERVOS)
    for i in range (NUM_SERVOS):
        servo_angles[i] = rospy.get_param("/initialize_servos/"+servo_param_names[i])
        servo_vels[i]   = rospy.get_param("/initialize_servos/"+angvel_param_names[i])


    for i in range(NUM_SERVOS):
        msg.goal_angles.append(servo_angles[i])
        msg.goal_vels.append(servo_vels[i])

    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()



   