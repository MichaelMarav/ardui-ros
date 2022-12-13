#include "ardui_driver/remapper.h"
#include <iostream>


/* 
 * Constructor
*/
// TODO: Initialize once all used msgs
Remapper::Remapper(ros::NodeHandle nh)
{   
    // Remapping params
    nh.getParam("/analog_min",analog_min);
    nh.getParam("/analog_max",analog_max);
    
    nh.getParam("/angles_min",angles_min);
    nh.getParam("/angles_max",angles_max);
    
    nh.getParam("/micro_min",micro_min);
    nh.getParam("/micro_max",micro_max);



    // Allocate memory to messages 
    joint_msg.angles.resize(this->NUM_SERVOS);

    analog2angles_msg.analog_feedback.resize(this->NUM_SERVOS);


}   


/*
 * The map() function from arduino 
 */
float Remapper::remap(const float & input,const float & in_min, const float & in_max,const float & out_min,const float & out_max)
{
    return static_cast<float>( (input-in_min)*(out_max-out_min)/(in_max-in_min) + out_min );
}


/* 
 * Input: AnalogFeedback msg from a teensy topic
 * Description: Maps analog feedback to angles and stores it to analog2angles_msg
 * Output: Returns the AnalogFeedback angles-mapped message 
 */
ardui_msgs::JointStates Remapper::analog2angles(const ardui_msgs::AnalogFeedback & msg)
{   
    analog2angles_msg.header = msg.header;
    for (int i = 0 ; i < this->NUM_SERVOS ; ++i){
        joint_msg.angles[i]  = remap(msg.analog_feedback[i],this->analog_min,this->analog_max,this->angles_min,this->angles_max);
    }
    return joint_msg;
} 



/* 
 * Input: GoalStates msg with the goal angles and goal velocities in degrees
 * Description: Remaps angles AND angular vel into microseconds 
 * Output: Returns the ardui_msgs::GoalStates in microseconds    
*/
void Remapper::GoalAngles2micro(ardui_msgs::GoalStates & msg)
{
    int N = this->NUM_SERVOS;
    for (int i = 0 ; i < N ; ++i){
        msg.goal_angles[i]  = this->remap(msg.goal_angles[i],angles_min,angles_max,micro_min,micro_max);
        msg.goal_vels[i]    = this->remap(msg.goal_vels[i],angles_min,angles_max,micro_min,micro_max) - micro_min; // 10 degrees/sec -> 560 which is wrong we need to subtrack micro_min
    }
}



/*
 * Converts the input msg measurements units 
 */
void Remapper::JointAngles2micro(ardui_msgs::JointStates & msg)
{
    int N = this->NUM_SERVOS;
    for (int i = 0 ; i < N ; ++i){
        msg.angles[i]  = this->remap(msg.angles[i],angles_min,angles_max,micro_min,micro_max);
    }
}

