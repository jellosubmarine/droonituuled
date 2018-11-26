#ifndef OFFB_CONTROLLER_INCLUDED
#define OFFB_CONTROLLER_INCLUDED

#include <string>

#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/VFR_HUD.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PointStamped.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/StreamRate.h"
#include "opencv2/core.hpp"

#include "dt_config.hpp"
#include "offb_controller.hpp"
#include "offb_pid.hpp"


class OffbController {
public:
  int init();
  void loop();  // Main flight loop
  // static void shutdown();

  // Visuals
  #ifdef OFFB_SHOW_VISUALS
    void initVisuals();
    void updateVisuals(double x, double y, double dir);
  #endif

  // Callbacks
  void stateCB(const mavros_msgs::State::ConstPtr& msg);
  void timeoutCB(const ros::TimerEvent&);
  void vfrCB(const mavros_msgs::VFR_HUD::ConstPtr& msg);
  void imuCB(const sensor_msgs::Imu::ConstPtr& msg);
  void camCB(const mavros_msgs::PositionTarget::ConstPtr& msg);

  // Ros Comms
  ros::NodeHandle nh;

  ros::ServiceClient mode_client;
  ros::ServiceClient arm_client;
  ros::ServiceClient stream_rate_client;

  ros::Subscriber stateSub;
  ros::Subscriber vfrSub;
  ros::Subscriber imuSub;
  ros::Subscriber camSub;

  ros::Publisher raw_pub;

  // Stream rate
  mavros_msgs::StreamRate srv;
  int stream_id;
  int rp_stream_rate;
  bool stream_on_off;

private:
  // Utilities
  template<typename T> void readParam(const std::string& param_name, T* var, const T& defaultVal);
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
  mavros_msgs::PositionTarget camData;
  sensor_msgs::Imu imuData;
  double zeroAlt;
  bool timeout;

  // Ros parameters
  float rp_lim_nav_alt, rp_lim_nav_vs;
  float rp_lim_abort_alt, rp_lim_abort_lost_time;
  float rp_pid_alt_target;
  float rp_pid_alt_out_min, rp_pid_alt_out_max;
  float rp_pid_alt_out_ramp, rp_pid_alt_out_bias;
  float rp_pid_alt_p, rp_pid_alt_i, rp_pid_alt_d;
  float rp_pid_pitch_target;
  float rp_pid_pitch_out_min, rp_pid_pitch_out_max, rp_pid_pitch_out_ramp;
  float rp_pid_pitch_p, rp_pid_pitch_d;
  float rp_pid_roll_target;
  float rp_pid_roll_out_min, rp_pid_roll_out_max, rp_pid_roll_out_ramp;
  float rp_pid_roll_p, rp_pid_roll_d;
  float rp_pid_yaw_target;
  float rp_pid_yaw_out_min, rp_pid_yaw_out_max, rp_pid_yaw_out_ramp;
  float rp_pid_yaw_p, rp_pid_yaw_d;
  float rp_roll_yaw_coupling;

  // Visuals
  #ifdef OFFB_SHOW_VISUALS
    void drawGrid(double step);
    cv::Mat displayMatrix;
  #endif

  // Debug
  void debug(double w, double x, double y, double z);
  ros::Publisher debug_pub;
  geometry_msgs::Quaternion debug_q;
};

#endif
