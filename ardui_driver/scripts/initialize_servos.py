#! /usr/bin/env python3
import rospy
from ecattie_msgs.msg import GoalStates
import numpy as np



NUM_SERVOS = 20
NUM_SERVOS_PER_FOOT = 5



if __name__ == "__main__":
    rospy.init_node('Initializer')
    msg = GoalStates()
    publish_topic_name = rospy.get_param("/servo_goal_topic")

    r = rospy.Rate(1) # 1hz


    pub = rospy.Publisher('/ecattie/servo_goal', GoalStates, queue_size=10)

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


    for i in range(NUM_SERVOS_PER_FOOT):
        msg.front_left.goal_angles.append(servo_angles[i])
        msg.front_left.goal_vels.append(servo_vels[i])

        msg.front_right.goal_angles.append(servo_angles[i+NUM_SERVOS_PER_FOOT])
        msg.front_right.goal_vels.append(servo_vels[i+NUM_SERVOS_PER_FOOT])

        msg.back_left.goal_angles.append(servo_angles[i + 2*NUM_SERVOS_PER_FOOT])
        msg.back_left.goal_vels.append(servo_vels[i + 2*NUM_SERVOS_PER_FOOT])

        msg.back_right.goal_angles.append(servo_angles[i+3*NUM_SERVOS_PER_FOOT])
        msg.back_right.goal_vels.append(servo_vels[i+3*NUM_SERVOS_PER_FOOT])

    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()



   