#include "ros/ros.h"
#include "ardui_driver/speed_controller.h"



int main(int argc, char **argv)
{
    ros::init(argc,argv,"ServoControlNode");
    ros::NodeHandle nh;
    SpeedController Speed_Controller(nh);
    
}