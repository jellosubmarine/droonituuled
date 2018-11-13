#ifndef OFFB_CONTROLLER_INCLUDED
#define OFFB_CONTROLLER_INCLUDED

#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/VFR_HUD.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "offb_pid.hpp"


class OffbController {
public:
  int init();
  void loop();  // Main flight loop
  static void shutdown();

  // Callbacks
  void stateCB (const mavros_msgs::State::ConstPtr& msg);
  void timeoutCB (const ros::TimerEvent&);
  void vfrCB (const mavros_msgs::VFR_HUD::ConstPtr& msg);
  void imuCB (const sensor_msgs::Imu::ConstPtr& msg);
  void floorCB (const geometry_msgs::QuaternionStamped::ConstPtr& msg);

  // Ros Comms
  ros::NodeHandle nh;

  ros::ServiceClient mode_client;
  ros::ServiceClient arm_client;

  ros::Subscriber stateSub;
  ros::Subscriber vfrSub;
  ros::Subscriber imuSub;
  ros::Subscriber floorSub;

  ros::Publisher raw_pub;

private:
  // Utilities
  int setMode(std::string modeName);
  int armVehicle();
  void zeroAltitude();
  ros::Timer startTimeout(double duration);

  // PID controllers
  OffbPID altPID;
  OffbPID pitchPID;
  OffbPID rollPID;
  OffbPID yawPID;

  // Data storage
  mavros_msgs::State currentState;
  mavros_msgs::VFR_HUD flightData;
  geometry_msgs::QuaternionStamped floorData;
  sensor_msgs::Imu imuData;
  double zeroAlt;
  bool timeout;

  // Debug
  void debug(double w, double x, double y, double z);
  ros::Publisher debug_pub;
  geometry_msgs::Quaternion debug_q;
};

#endif
