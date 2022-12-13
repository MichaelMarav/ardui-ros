#pragma once

#include "ardui_driver/remapper.h"
#include "string.h"



class Driver{
    private:
       
        // Subscribing topic names
        std::string analog_topicname;
        // Publishing topic names
        std::string joint_topicname; 

        // ROS Subscriber for analog feedback that is being sent from microcontroller
        ros::Subscriber analog_sub;

        // ROS Publisher: JointStates publisher
        ros::Publisher joint_pub;

        ardui_msgs::AnalogFeedback analog_msg;
        ardui_msgs::GoalStates servo_goal_msg;
        ardui_msgs::JointStates joint_msg;
        
        bool analog_flag;

        Remapper remapper = Remapper(ros::NodeHandle());


        // Subscribing callbacks
        void analog_callback(const ardui_msgs::AnalogFeedback::ConstPtr & msg);     

    public:

        // Constructor
        Driver(ros::NodeHandle nh);

        // Main subscribe/publish loop
        void ros_loop();   
};

