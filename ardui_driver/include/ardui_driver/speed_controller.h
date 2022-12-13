#pragma once

#include "ardui_driver/remapper.h"

#include "ardui_msgs/ServoCommand.h"
#include "ardui_msgs/GoalStates.h"


class SpeedController
{
    private:
        std::string sub_goal_topic;
        std::string pub_micro_topic; // Microcommands for teensy
        std::string sub_joint_topic;
        // Publishers        
        ros::Publisher microgoal_pub;

        // Subscriber
        ros::Subscriber servo_goal_sub;
        ros::Subscriber joint_sub; // To get initial angle of servo
        // Messages
        ardui_msgs::GoalStates goal_msg;

        ardui_msgs::JointStates joint_msg;
        ardui_msgs::JointStates initial_joints_msg;

        ardui_msgs::ServoCommand final_pos_msg; // Front final goal angle command in microseconds 

        ardui_msgs::ServoCommand microgoal_msg;  // Back micro-commands

        // Remapper object
        Remapper remapper = Remapper(ros::NodeHandle());


        bool goal_arrived_flag{false};
        bool joint_states_flag;
        int ros_rate_value;
        int angular_vel;
        
        ros::Rate loop_rate = ros::Rate(1);   // Random initializer value = 1


        // Subscribing Callback
        void servo_goal_callback(const ardui_msgs::GoalStates & msg);
        void joint_states_callback(const ardui_msgs::JointStates::ConstPtr & msg);

        // Allocates memory to ros msgs
        void initialize_messages();

        // Main Subscribing/Publishing ros loop
        void ros_loop();

    public:
        // Constructor
        SpeedController(ros::NodeHandle nh);
};  