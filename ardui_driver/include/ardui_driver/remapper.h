#pragma once             // Same as ifndef etc.
//#ifndef REMAPPER_H_ // Header guards
//#define REMAPPER_H_


// All messages
#include "ardui_msgs/AnalogFeedback.h"
#include "ardui_msgs/JointStates.h"
#include "ardui_msgs/GoalStates.h"
#include "ardui_msgs/ServoCommand.h"
#include "ros/ros.h"

#include <vector>


class Remapper
{
    private:
        // Mapping parameters
        int analog_min, analog_max, angles_min, angles_max, micro_min, micro_max; 


        ardui_msgs::AnalogFeedback analog2angles_msg;
        ardui_msgs::JointStates joint_msg;     // Stores the servo positions in angles
       
        float remap(const float & input,const float & in_min,const float & in_max,const float & out_min,const float & out_max);

    
    public:
        const int NUM_SERVOS{3};

        // Constructor
        Remapper(ros::NodeHandle nh);
        
        // Mappping functions
        ardui_msgs::JointStates analog2angles(const ardui_msgs::AnalogFeedback & msg);
        void GoalAngles2micro(ardui_msgs::GoalStates & msg);
        void JointAngles2micro( ardui_msgs::JointStates & msg);

};


// #endif