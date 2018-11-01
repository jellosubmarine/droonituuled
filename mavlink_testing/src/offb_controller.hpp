#ifndef OFFB_CONTROLLER_INCLUDED
#define OFFB_CONTROLLER_INCLUDED

#include "ros/ros.h"
#include "mavros_msgs/State.h"


class OffbController {
public:
  // Callbacks
  void state_cb(const mavros_msgs::State::ConstPtr& msg);
  void timeout_cb(const ros::TimerEvent&);

  // Utilities
  int setup();
  int setMode(std::string mode_name);
  int armVehicle();
  ros::Timer startTimeout(double duration);

  // Fields
  ros::NodeHandle nh;
  ros::ServiceClient mode_client;
  ros::ServiceClient arm_client;
  ros::Subscriber state_sub;

  mavros_msgs::State current_state;
  bool timeout;

private:
  //
};

#endif
