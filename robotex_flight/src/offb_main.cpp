// #include <csignal>

#include "ros/ros.h"
#include "ros/time.h"
// #include "mavros_msgs/ParamSet.h"
// #include "mavros_msgs/SetMode.h"

#include "offb_controller.hpp"
#include "offb_config.hpp"
#include "dt_config.hpp"


#ifdef DT_BUILD_DEV
  #pragma message "robotex_flight dev build"
#else
  #pragma message "robotex_flight live build"
#endif


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
  // ros::init(argc, argv, "Droonituuled", ros::init_options::NoSigintHandler);
  ros::init(argc, argv, "robotex_flight");
  // ROS_INFO("Inited robotex_flight");

  ros::NodeHandle nh;
  // ros::ServiceClient param_cl = nh.serviceClient<mavros_msgs::ParamSet> ("/mavros/param/set");
  // mavros_msgs::ParamSet paramMsg;

  // signal(SIGINT, sigintHandler);

  // Set up controller
  OffbController ctrl;
  ctrl.nh = nh;

  #ifdef DT_BUILD_LIVE
    ros::Duration(6, 0).sleep();
  #endif

  // ROS_INFO("Initialising controller");
  if (ctrl.init()) {
    // ROS_INFO("Starting controller loop");
    ctrl.loop();
  } else {
    ROS_INFO("Failed to initialise flight controller");
  }

  ROS_INFO("robotex_flight finished");
}
