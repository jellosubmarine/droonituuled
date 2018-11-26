#include <csignal>
// #include <cstdio>
#include <iostream>

#include "ros/ros.h"
#include "ros/time.h"
#include "mavros_msgs/StreamRate.h"
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


volatile int g_flight_exit = 0;

void sigintHandler(int sig) {
  std::cout << "Sigint Handler";
  g_flight_exit = 1;
}

int setStreamRate(ros::NodeHandle *nh, const OffbController &ctrl) {
  ros::ServiceClient stream_rate_client = nh->serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
  mavros_msgs::StreamRate rateMsg;

  rateMsg.request.stream_id = 0;
  rateMsg.request.message_rate = ctrl.rp_stream_rate;
  rateMsg.request.on_off = 1;
  if (!stream_rate_client.call(rateMsg)) {
    ROS_INFO("Failed to set Pixhawk stream rate");
    return 0;
  }

  return 1;
}

/* ============ MAIN ============ */

int main(int argc, char **argv) {
  ros::init(argc, argv, "robotex_flight");
  ros::NodeHandle nh;
  // ros::ServiceClient param_cl = nh.serviceClient<mavros_msgs::ParamSet> ("/mavros/param/set");
  // mavros_msgs::ParamSet paramMsg;

  // Set up controller
  OffbController ctrl;
  ctrl.nh = nh;

  #ifdef DT_BUILD_LIVE
    ros::Duration(6, 0).sleep();
  #endif

  signal(SIGINT, sigintHandler);

  if (ctrl.init() && setStreamRate(&nh, ctrl)) {
    while (!g_flight_exit) {
      ctrl.prepStandby();
      if (ctrl.prepFlight()) {
        ctrl.loop();
      }
    }
  } else {
    ROS_INFO("Failed to initialise flight controller");
  }

  ROS_INFO("robotex_flight finished");
}
