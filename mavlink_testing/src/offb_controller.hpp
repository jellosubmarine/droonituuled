#ifndef OFFB_CONTROLLER_INCLUDED
#define OFFB_CONTROLLER_INCLUDED

#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/VFR_HUD.h"

#include "offb_pid.hpp"


class OffbController {
public:
  int init();
  void loop();  // Main flight loop

  // Callbacks
  void state_cb(const mavros_msgs::State::ConstPtr& msg);
  void timeout_cb(const ros::TimerEvent&);
  void vfr_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg);

  // Ros Comms
  ros::NodeHandle nh;

  ros::ServiceClient mode_client;
  ros::ServiceClient arm_client;

  ros::Subscriber state_sub;
  ros::Subscriber vfr_sub;

  ros::Publisher raw_pub;
  ros::Publisher debug_pub;

private:
  // Utilities
  int setMode(std::string modeName);
  int armVehicle();
  void zeroAltitude();
  ros::Timer startTimeout(double duration);

  // PID controllers
  OffbPID altPID;

  // Data storage
  mavros_msgs::State currentState;
  mavros_msgs::VFR_HUD flightData;
  double zeroAlt;
  bool timeout;
};

#endif
