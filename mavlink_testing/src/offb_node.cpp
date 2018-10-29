/***************************************************************************
 *                       Varanon Austin Pukasamsombut					   *
 *																		   *
 *                  AERO = Autonomous Exploration RObot                    *
 *                                 Test                                    *
 *                                                                         *
 * This code is designed to make a Quadcopter (with APM 2.6) autonomously  *
 * takeoff in ALT_HOLD mode, hover in the air for 20 seconds, and then     *
 * land by changing to LAND mode.                                          * 
 ***************************************************************************/

//Standard Include
#include "ros/ros.h"
#include <cstdlib>

//Include Message Types Used
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/FluidPressure.h>

//Define Radio Channels to Numbers [May differ depending on RC Transmitter]
#define ROLL 0
#define PITCH 1
#define THROTTLE 2
#define YAW 3
#define LEFT_TRIGGER 4

//Define Checks
#define PRELIMINARY_CHECK 1
#define LAND_CHECK 2

//Define RC Throttle Values [Differs for all controllers]
#define MIDDLE 1500  //PWM Value with Stick at Middle 
#define RISING 1690  //PWM Value with Stick Slightly Above Middle
#define RELEASE 1100 //PWM Value when Stick is completely Lowered
#define NO_RC 900	 //PWM Value when no RC Controller has been connected.

#define HOVER_H 2    // Hovering height (no idea what units)

//Class Define (Maybe Move to Separate File Later)
class Receiver
{
    public:
        //Callbacks
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);
        void vfrCallback(const mavros_msgs::VFR_HUD::ConstPtr& msg);
        void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);
        
        //Looping Checks
        bool state_finished; //If False, then runs stateCallback on spin.
        bool vfr_finished;
        bool rc_finished;
        bool terminate; //Release All Channels and Terminate Program
        bool get_alt;

        //Miscellaneous Variables
        int state_check;
        int vfr_check;
        int rc_check_ch; //Channel to Check
        int rc_check_val; //Value to Check

        float initial_alt;
        
};

void Receiver::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    ROS_INFO("ENTER: State Callback");
    if(terminate) return;
    if(state_finished) return;

    bool check1, check2;

    if(state_check == PRELIMINARY_CHECK)
    {
        if(msg->mode == "ALT_HOLD") 
        {
            ROS_INFO("Completed: ALT_HOLD");
            check1 = true;
        }
        else 
        {
            ROS_INFO("Waiting: ALT_HOLD");
            check1 = false;
            system("rosrun mavros mavsys mode -c ALT_HOLD");
        }
        if(msg->armed)
        {
            ROS_INFO("Completed: ARMING");
            check2 = true;
        }
        else
        {
            ROS_INFO("Waiting: ARMING");
            check2 = false;
            system("rosrun mavros mavsafety arm");
        }
    
        if(check1 && check2) 
        {
            ROS_INFO("Completed: Preliminary Check");
            state_finished = true;
        }
    }//End Preliminary Check

    if(state_check == LAND_CHECK)
    {
        if(msg->mode == "LAND") 
        {
            ROS_INFO("Completed: LAND");
            state_finished = true;
        }
        else 
        {
            ROS_INFO("Waiting: LAND");
            system("rosrun mavros mavsys mode -c LAND");
        }
    }//End Landing Check
}
    
void Receiver::vfrCallback(const mavros_msgs::VFR_HUD::ConstPtr& msg)
{
    ROS_INFO("Enter: VFR Callback");

    if (get_alt == true)
    {
        initial_alt = msg->altitude;
        get_alt = false;
    }


    if(terminate) return;
    if(vfr_finished) return;
    
    if(vfr_check == RISING)
    {
        if(msg->altitude < initial_alt+HOVER_H)
        {
            ROS_INFO("Waiting: RISING");
            vfr_finished = false;
        }
        else //Rises until the Altitude Goes Above 2.6
        {
            ROS_INFO("Completed: RISING");
            vfr_finished = true;
        }
    }
}


void Receiver::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    ROS_INFO("Enter: RC Callback");

    //Terminates all Programs if Left Trigger Moves: "Forced Landing" [Differs for All Controllers]
    if(msg->channels[LEFT_TRIGGER] > 1800) terminate = true;

    if(rc_finished) return;

    if(msg->channels[rc_check_ch] <= (rc_check_val + 7)
            ||msg->channels[rc_check_ch] >= (rc_check_val - 7)) //RC Values Fluctuates, so +-7 takes that into account.
    {
        rc_finished = true; //If RC_Value is close to Required Value.
        ROS_INFO("Completed: RC Change");
    }
    else 
        rc_finished = false;   
}


/***********************MAIN PROGRAM BEGINS HERE****************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aero_test"); //Initialization
    ros::NodeHandle nh; //Way to Connect Node to Master

    ROS_INFO("AERO Program: Test");

    //Setting Up All Subscribers and Publishers

    Receiver receiver;
    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 1, &Receiver::stateCallback, &receiver);
    ros::Subscriber vfr_sub = nh.subscribe("/mavros/vfr_hud", 1, &Receiver::vfrCallback, &receiver);
    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 1, &Receiver::rcCallback, &receiver);

    receiver.state_finished = true;
    receiver.vfr_finished = true;
    receiver.rc_finished = true;
    receiver.terminate = false;
    receiver.get_alt = true;

    ros::Publisher rc_message = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1, true);
    mavros_msgs::OverrideRCIn rc_command;

    //First Arm and Set to Altitude_Hold Mode
    ROS_INFO("Commencing: Preliminary Setup");    
    
    system("rosrun mavros mavsys mode -c ALT_HOLD");
    
    system("rosrun mavros mavsafety arm");
    
    receiver.state_check = PRELIMINARY_CHECK;
    receiver.state_finished = false; //Calls State Callback
    while((!receiver.state_finished) && (ros::ok()) && (!receiver.terminate))
        ros::spinOnce();

    ROS_INFO("Commencing: Take Off");
    
    //Then Push Throttle Up to Fly, Leaving All Others to RC Controller
    
    for(int i=0; i < 8; i++) rc_command.channels[i] = 0;//Releases all Channels First
    rc_command.channels[THROTTLE] = RISING; //Ascending Throttle

    receiver.rc_check_ch = THROTTLE;
    receiver.rc_check_val = RISING;
    receiver.rc_finished = false;
    while((!receiver.rc_finished) && (ros::ok()) && (!receiver.terminate)) 
    {
        ros::spinOnce();
        rc_message.publish(rc_command);
    }

    ROS_INFO("Completed: Take Off");
   
    //Continues until Altitude Reaches Certain Point (+ 1m)

    ROS_INFO("Commencing: Ascent");

    //receiver.landing = false;
    //receiver.alt_ground = 0.00;

    receiver.vfr_check = RISING;
    receiver.vfr_finished = false;
    while((!receiver.vfr_finished) && (ros::ok()) && (!receiver.terminate)) 
    {
        ros::spinOnce();
        rc_message.publish(rc_command);
    }
        

    ROS_INFO("Completed: Ascent");

    //Then Sets Throttle to Middle Value and Keeps Position for Set Time
    
    ROS_INFO("Commencing: Hover");

    rc_command.channels[THROTTLE] = MIDDLE; //Keeps Altitude
    
    receiver.rc_check_ch = THROTTLE;
    receiver.rc_check_val = MIDDLE;
    receiver.rc_finished = false;
    while((!receiver.rc_finished) && (ros::ok()) && (!receiver.terminate))
    {
        ros::spinOnce();
        rc_message.publish(rc_command);
    }

    ros::Time begin = ros::Time::now();
    double secs = 0; //Set to Hover for 20 seconds
    while((secs < 20) && (ros::ok()) && (!receiver.terminate))
    {
        ros::spinOnce();
        ros::Duration waiting = ros::Time::now() - begin;
        secs = waiting.toSec();
        ROS_INFO("Waiting: Hover");
    }

    ROS_INFO("Completed: Hover");

    //Changes mode to LAND and Begins Landing
    
    ROS_INFO("Commencing: Landing");
   
    if((!receiver.terminate) && (ros::ok())) 
        system("rosrun mavros mavsys mode -c LAND");    
    receiver.state_check = LAND_CHECK;
    receiver.state_finished = false; //Calls State Callback
    while((!receiver.state_finished) && (ros::ok()) && (!receiver.terminate))
        ros::spinOnce();

    ROS_INFO("Commencing: Throttle Release");

    //Releases RC Override Controls and Exits Program
   
    for(int i=0; i < 8; i++) rc_command.channels[i] = 0;

    receiver.rc_check_ch = THROTTLE;
    receiver.rc_check_val = RELEASE;
    receiver.rc_finished = false;
    while((!receiver.rc_finished) && (ros::ok())) 
    {
        ros::spinOnce();
        rc_message.publish(rc_command);
    }
    
    ROS_INFO("Completed: Landing");

    if(!receiver.terminate)
    {
        system("rosrun mavros mavsafety disarm");
        ROS_INFO("Completed: DISARMED");
        ROS_INFO("Program Completed");
    }
    if(receiver.terminate)
        ROS_INFO("FORCED TERMINATION INVOKED");

    ROS_INFO("Terminating Code");

    return 0;
}





/* *
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

/* #include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_rc_override");
    ros::NodeHandle n;


    int rate = 100;
    ros::Rate r(rate);

    ros::Publisher rc_override_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", -1);

    mavros_msgs::OverrideRCIn msg_override;

    while (n.ok()){

        msg_override.channels[0] = 1500;
        msg_override.channels[1] = 1500;
        msg_override.channels[2] = 1550;
        msg_override.channels[3] = 1500;
        msg_override.channels[4] = 1500;
        msg_override.channels[5] = 1500;
        msg_override.channels[6] = 1500;
        msg_override.channels[7] = 1500;

        rc_override_pub.publish(msg_override);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
 */


/* #include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // RC publisher
    ros::Publisher pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);
    ROS_INFO("Test");
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("test2");


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "MANUAL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    mavros_msgs::OverrideRCIn msg;

    msg.channels[5] = 1600; // forward  (x)
    msg.channels[6] = 1900; // strafe   (y)
    msg.channels[2] = 1550; // throttle (z)

    msg.channels[1] = 1900; // roll     (wx)
    msg.channels[0] = 1900; // pitch    (wy)
    msg.channels[3] = 1900; // yaw      (wz)

    msg.channels[4] = 1900; // mode
    msg.channels[7] = 1900; // camera tilt
    pub.publish(msg);
    while(ros::ok()){

        if( current_state.mode != "MANUAL" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Manual enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

    

    ros::spinOnce();
    rate.sleep();
    }

    return 0;
} */
