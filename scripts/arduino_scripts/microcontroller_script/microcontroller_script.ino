/*
 * Generic Libraries
 */
#include <Arduino.h>
#include <ros.h>
#include <Servo.h>

/*
 * Custom messages
 */
#include <ardui_msgs/ServoCommand.h>
#include <ardui_msgs/AnalogFeedback.h>



/* 
 *  SERVO PARAMETERS DEFINITIONS =====> CHANGE THE FOLLOWING FOR DIFFERENT TEENSY
 */
// --------------------------------------------------------------------------------------------------------------------------------------------
#define NUM_SERVOS 3
Servo servo[NUM_SERVOS];
short int SERVO_PINS[NUM_SERVOS] = {0,1,2};
uint8_t FEEDBACK_PINS[NUM_SERVOS] = {A9,A8,A7};
// --------------------------------------------------------------------------------------------------------------------------------------------


 

/*
 * ROS Definitions
 */
// --------------------------------------------------------------------------------------------------------------------------------------------
// Subscriber callback
void servo_callback(const ardui_msgs::ServoCommand& goal_msg )
{
        for (int i = 0 ; i < NUM_SERVOS ; ++i){
            servo[i].writeMicroseconds(goal_msg.micro_command[i]); 
        }
}

ros::NodeHandle nh;

// Global msgs
ardui_msgs::ServoCommand goal_msg;                    // Subscriber msg
ardui_msgs::AnalogFeedback  feedback_msg;          // Publisher msg

ros::Subscriber<ardui_msgs::ServoCommand> goal_sub("/teensy/microgoal",servo_callback);     // Subscriber
ros::Publisher joint_pub("/teensy/analog_feedback", &feedback_msg);                                           // Publsiher
// --------------------------------------------------------------------------------------------------------------------------------------------




/* 
 *  Initialize ROS messages (allocate memory) 
 */
void initialize_array_msg()
{
        
    feedback_msg.analog_feedback = new float[NUM_SERVOS]();
    feedback_msg.analog_feedback_length  = NUM_SERVOS;
    
    goal_msg.micro_command = new long unsigned int[NUM_SERVOS]();
    goal_msg.micro_command_length = NUM_SERVOS;
}
// --------------------------------------------------------------------------------------------------------------------------------------------







/*
 * Get analog feedback for every servo and return it as is
 */
// --------------------------------------------------------------------------------------------------------------------------------------------
void  get_analog_feedback()
{
    // Read feedback 
    for (int s = 0 ; s < NUM_SERVOS; ++s){
        feedback_msg.analog_feedback[s]  = analogRead(FEEDBACK_PINS[s]);
    }
}
// --------------------------------------------------------------------------------------------------------------------------------------------






// --------------------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
    Serial.begin(57600); // This baud rate is set by default at serial_node.py 57600

    /*
     * Initialize ROS 
     */
    nh.initNode(); 
    nh.subscribe(goal_sub);
    nh.advertise(joint_pub); 


    initialize_array_msg();


    /* 
     *  Attach servos and write their current positions
     */
    for (int s = 0 ; s < NUM_SERVOS; ++s){
        pinMode(FEEDBACK_PINS[s],INPUT);    
        servo[s].attach(SERVO_PINS[s],500,2400); // Min and max range in microseconds 
        servo[s].writeMicroseconds(map(analogRead(FEEDBACK_PINS[s]), 73 ,998,2400, 500 ) ); // Remap analog read to microseconds
    }

}
// --------------------------------------------------------------------------------------------------------------------------------------------



/* 
 *  MAIN LOOP
 */
// --------------------------------------------------------------------------------------------------------------------------------------------

void loop() 
{
    nh.spinOnce();
    get_analog_feedback();
    joint_pub.publish(&feedback_msg);
    delayMicroseconds(700);
}
// --------------------------------------------------------------------------------------------------------------------------------------------
