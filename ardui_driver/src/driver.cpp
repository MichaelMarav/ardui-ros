#include "ardui_driver/driver.h"


/*
 * Driver Constructor
 */
Driver::Driver(ros::NodeHandle nh):remapper(nh)
{
    // Get parameters from parameter server
    nh.getParam("/analog_feedback_topic",analog_topicname);
    nh.getParam("/joint_states_topic",joint_topicname);

    // Initialize Subscribers
    analog_sub = nh.subscribe(analog_topicname,1000,&Driver::analog_callback,this);

    // Initialize Publishers
    joint_pub  = nh.advertise<ardui_msgs::JointStates>(joint_topicname,1);



    joint_msg.angles.resize(remapper.NUM_SERVOS);
      
    ros_loop();
}



/* 
 * Input: Address for msg from the front teensy topic 
 * Description: front teensy analog callback. stores the value and changes the flag
 */
void Driver::analog_callback(const ardui_msgs::AnalogFeedback::ConstPtr & msg)
{
    analog_msg = *msg;
    this->analog_flag = true;
}




/*
 * Description: Main ros loop. Subscribe/Publish to ROS topics
 */
void Driver::ros_loop()
{
    while (ros::ok()){
        ros::spinOnce();


        // TODO: Check if needed to split the below commands into functions
        if (this->analog_flag){ // Check if new messages from both topics arrived
            // Map analog read to angles
            joint_msg = remapper.analog2angles(analog_msg);

            // Merge and publish the joint states
            joint_pub.publish(joint_msg);

            // drop flag to get a new msg
            analog_flag = false;
        }

    }   
}



