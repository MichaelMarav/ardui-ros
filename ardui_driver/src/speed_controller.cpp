#include "ardui_driver/speed_controller.h"


/*
 * Driver Constructor
 */
SpeedController::SpeedController(ros::NodeHandle nh):remapper(nh)
{   


    // Get subscribing topic names
    nh.getParam("/servo_goal_topic",sub_goal_topic);
    nh.getParam("/joint_states_topic",sub_joint_topic);

    // Get publishing topics names
    nh.getParam("/subgoal_topic",pub_micro_topic);

    // Ros::rate (Unusable for now (let it run at max speed)
    nh.getParam("/ros_rate",ros_rate_value);


    // Publishers
    microgoal_pub = nh.advertise<ardui_msgs::ServoCommand>(pub_micro_topic,1);
    // Subscribers
    servo_goal_sub = nh.subscribe(sub_goal_topic,1,&SpeedController::servo_goal_callback,this);
    joint_sub      = nh.subscribe(sub_joint_topic,1,&SpeedController::joint_states_callback,this);
    // Define loop_rate
    loop_rate = ros::Rate(ros_rate_value);    

    initialize_messages();

    this->ros_loop();

}


/*
 * Allocate memory to all ROS msgs and initialize prev_goal_msg to a dummy value that the the goal_msg can't get 
 */
void SpeedController::initialize_messages()
{

    // TODO: Check if header is required for these messages
    int N = remapper.NUM_SERVOS;
    // Initialize goal msg
    this->goal_msg.goal_angles.resize(N);
    this->goal_msg.goal_vels.resize(N);
    

    // Initialize previous goal message (with values because when you want to go to zero you got a problem)
   
    // Initialize final goal for front and back teensys
    this->final_pos_msg.micro_command.resize(N);

    // Initilize microgoal msgs
    this->microgoal_msg.micro_command.resize(N);
   
    // Joint States msg
    this->joint_msg.angles.resize(N);
    this->joint_msg.effort.resize(N);

    // Stores the initial position of each servo 
    this->initial_joints_msg.angles.resize(N);

}




/*
 * Final goal callback function. Gets the goal message (in angles) and also checks if 
 * the previous goal point is the same as the current one and raises a flag. Also converts the angles/sec to micro/sec
 */
void SpeedController::servo_goal_callback(const ardui_msgs::GoalStates & msg)
{   
    goal_msg = msg;
    goal_arrived_flag = true;
}




/*
 * Joint states Callback, PLACEHOLDER: use joint_msg for closed loop control 
 */
void SpeedController::joint_states_callback(const ardui_msgs::JointStates::ConstPtr & msg)
{
    joint_msg = *msg; // Save initial angle of servos ( use joint_msg and not initial_joint_msg) as a placeholder for implementing the closed loop
    joint_states_flag = true;
}


/*
 * Main ros loop
 */

void SpeedController::ros_loop()
{


    // ros::Time t1,t2;
    while (ros::ok()){
        ros::spinOnce();

        if (goal_arrived_flag && joint_states_flag){
            // Individual flag for every servo to check if goal is achieved
            std::vector<bool> arrived_flag(remapper.NUM_SERVOS,false);



            // Drop flags
            goal_arrived_flag = false;
            joint_states_flag = false;

            // Converts the angles to microseconds
            remapper.GoalAngles2micro(goal_msg);

            // Convert JointAngles to Micro to get initial position
            remapper.JointAngles2micro(joint_msg);

            // Initial angles of servos in microseconds
            initial_joints_msg = joint_msg;

            // Fix the rotational speed sign for every servo && save the initial position
            for (int i = 0 ; i < remapper.NUM_SERVOS ; ++i){

                if (joint_msg.angles[i] > final_pos_msg.micro_command[i]){
                    goal_msg.goal_vels[i] *= -1.;
                }
            }
            





            double t1 = ros::Time::now().toSec();
            while (ros::ok()){

                ros::spinOnce();
                // If new goal arrived break the loop and start planning again
                if (goal_arrived_flag){ 
                    ROS_INFO("Replanning.. New goal arrived\n");
                    break;
                }



                double t2 = ros::Time::now().toSec();
                for (int i = 0 ; i < remapper.NUM_SERVOS ; ++i){        
                    // micro_goal = dt*w + start_angles
                    // Don't comput new message if the goal is achieved for each servo
                    if (( goal_msg.goal_vels[i] > 0 && microgoal_msg.micro_command[i] < final_pos_msg.micro_command[i]) 
                       || (goal_msg.goal_vels[i] < 0 && microgoal_msg.micro_command[i] > final_pos_msg.micro_command[i]) ) {
                        microgoal_msg.micro_command[i]  = std::round( (t2-t1)*goal_msg.goal_vels[i]  + initial_joints_msg.angles[i]   );
                    }else{
                        arrived_flag[i] = true; 
                    }
                   
                }

                microgoal_pub.publish(microgoal_msg);

                // Condition to stop for ardui
                if (std::all_of(arrived_flag.cbegin(),arrived_flag.cend(),[](bool i){return i;}))
                {
                    ROS_INFO("-- GOAL ACHIVED --\n Total Elapsed time: %f",t2-t1);
                    // std::cout << "Total elapsed time  = " << t2 - t1 << '\n';
                    break;
                }
                

                
                this->loop_rate.sleep(); // If not used, the communication with teensy breaks
            }
            
            
        } 
    }
}