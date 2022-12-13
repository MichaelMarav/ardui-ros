#! /usr/bin/env python3

import rospy
from ecattie_msgs.msg import AnalogFeedback



if __name__ == "__main__":
    rospy.init_node('dummy_pub')
    test_pub = rospy.Publisher("/teensy_back/analog_feedback", AnalogFeedback, queue_size=1)
    dummy_msg = AnalogFeedback()
    for i in range(10):
        dummy_msg.left_analog.append(43)
        dummy_msg.right_analog.append(43)
    rate = rospy.Rate(700)


    while not rospy.is_shutdown():
        test_pub.publish(dummy_msg)
        rate.sleep()
