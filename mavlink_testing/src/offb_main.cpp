#include <csignal>

#include "ros/ros.h"
#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/SetMode.h"

#include "offb_controller.hpp"
//#include "offb_config.hpp"

#define AHRS_GPS_USE_DISABLED 0
#define AHRS_GPS_USE_ENABLED  1
#define EK2_GPS_TYPE_INHIBIT  3
#define EK2_GPS_TYPE_ALL      0
#define GPS_TYPE_NONE         0
#define GPS_TYPE_AUTO         1


void sigintHandler(int sig) {
	/*
	ros::NodeHandle nh;
	ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
	mavros_msgs::SetMode modeCmd;
	modeCmd.request.custom_mode = "LAND";
	if (ros::ok()) mode_client.call(modeCmd);
	*/
}

/* ============ MAIN ============ */

int main(int argc, char **argv) {
	//ros::init(argc, argv, "Droonituuled", ros::init_options::NoSigintHandler);
	ros::init(argc, argv, "Droonituuled");
	ROS_INFO("Inited Droonituuled");

	ros::NodeHandle nh;
	//ros::ServiceClient param_cl = nh.serviceClient<mavros_msgs::ParamSet> ("/mavros/param/set");
	//mavros_msgs::ParamSet paramMsg;

	//signal(SIGINT, sigintHandler);

	ROS_INFO("Disabling GPS");
	/*
	paramMsg.request.param_id = "AHRS_GPS_USE";
	paramMsg.request.value.integer = AHRS_GPS_USE_DISABLED;
	param_cl.call(paramMsg);
	*/

	//paramMsg.request.param_id = "EK2_GPS_TYPE";
	//paramMsg.request.value.integer = EK2_GPS_TYPE_INHIBIT;
	//param_cl.call(paramMsg);

	/*
	paramMsg.request.param_id = "GPS_TYPE";
	paramMsg.request.value.integer = GPS_TYPE_NONE;
	param_cl.call(paramMsg);

	paramMsg.request.param_id = "GPS_TYPE2";
	paramMsg.request.value.integer = GPS_TYPE_NONE;
	param_cl.call(paramMsg);
	*/

	// Set up controller
	OffbController ctrl;
	ctrl.nh = nh;

	ROS_INFO("Initialising controller");
	if (ctrl.init()) {
		ROS_INFO("Starting controller loop");
		ctrl.loop();
	}

	ROS_INFO("Node Droonituuled finished");
}
