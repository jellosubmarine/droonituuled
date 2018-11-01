//#include "cstdlib"
#include <cmath>

#include "ros/ros.h"
#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/AttitudeTarget.h"

//#include "offb_math.hpp"
#include "offb_controller.hpp"

#define AHRS_GPS_USE_DISABLED 0
#define AHRS_GPS_USE_ENABLED  1
#define EK2_GPS_TYPE_INHIBIT  3
#define EK2_GPS_TYPE_ALL      0
#define GPS_TYPE_NONE         0
#define GPS_TYPE_AUTO         1

#define START_DELAY 0.5
#define LOOP_RATE	10




/* ============ MAIN ============ */

int main(int argc, char **argv) {
	ros::init(argc, argv, "Droonituuled");
	ROS_INFO("Inited Droonituuled");

	OffbController ctrl;

	// Set up topics
	ros::NodeHandle nh;
	ctrl.nh = nh;
	ctrl.setup();

	ros::Publisher raw_pub = nh.advertise<mavros_msgs::AttitudeTarget> ("/mavros/setpoint_raw/attitude", 3, true);

				//ros::ServiceClient param_cl = nh.serviceClient<mavros_msgs::ParamSet> ("/mavros/param/set");
				//mavros_msgs::ParamSet paramMsg;

				//ROS_INFO("Disabling GPS");
				/*
				paramMsg.request.param_id = "AHRS_GPS_USE";
				paramMsg.request.value.integer = AHRS_GPS_USE_DISABLED;
				param_cl.call(paramMsg);
				*/
				/*
				paramMsg.request.param_id = "EK2_GPS_TYPE";
				paramMsg.request.value.integer = EK2_GPS_TYPE_INHIBIT;
				param_cl.call(paramMsg);

				paramMsg.request.param_id = "GPS_TYPE";
				paramMsg.request.value.integer = GPS_TYPE_NONE;
				param_cl.call(paramMsg);

				paramMsg.request.param_id = "GPS_TYPE2";
				paramMsg.request.value.integer = GPS_TYPE_NONE;
				param_cl.call(paramMsg);
				*/

	ros::spinOnce(); // Refresh all callbacks
	ros::Duration start_delay(START_DELAY);

	// Connect to FCU
	ROS_INFO("Waiting for FCU connection");
	while (ros::ok() && !ctrl.current_state.connected) {
		ros::spinOnce();
		start_delay.sleep();
	}
	if (ros::ok() && ctrl.current_state.connected) ROS_INFO("Connected to FCU");

	// Enter stabilize mode
	if (!ctrl.setMode("STABILIZE")) return 0;
	// Arm vehice
	if (!ctrl.armVehicle()) return 0;

	// Set guided_nogps mode
	if (!ctrl.setMode("GUIDED_NOGPS")) return 0;

	ROS_INFO("Publishing initial setpoint");
	mavros_msgs::AttitudeTarget rawAtt;
	//rawAtt.thrust = 0.6;
	//rawAtt.orientation.x = sin(20.0*M_PI/180.0/2.0) * cos(20.0*M_PI/180.0);
	//rawAtt.orientation.w = cos(20.0*M_PI/180.0/2.0);
	//rawAtt.type_mask = rawAtt.IGNORE_THRUST;


	/* MAIN LOOP */

	ros::Rate loop_rate(LOOP_RATE);
	ros::Time startTime = ros::Time::now();
	ros::Duration end(12);
	ros::Duration step1(2);
	ros::Duration step2(4);
	ros::Duration step3(6);
	ros::Duration step4(8);
	ros::Duration step5(9);
	ros::Time t = ros::Time::now();

	while (ros::ok() && t - startTime < end) {
		t = ros::Time::now();

		if (t - startTime < step1) {
			rawAtt.thrust = 0.6;
		}
		else if (t - startTime < step2) {
			rawAtt.thrust = 1.0;
		}
		else if (t - startTime < step3) {
			rawAtt.thrust = 0.5;
		}
		else if (t - startTime < step4) {
			rawAtt.thrust = 0.3;
		}
		else if (t - startTime < step5) {
			rawAtt.thrust = 0.7;
		}
		else {
			rawAtt.thrust = 0.5;
		}

		raw_pub.publish(rawAtt);
		ros::spinOnce();
		loop_rate.sleep();
	}

	ctrl.setMode("LAND");
}
