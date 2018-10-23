/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
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

        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
