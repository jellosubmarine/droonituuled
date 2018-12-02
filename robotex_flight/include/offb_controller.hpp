#ifndef OFFB_CONTROLLER_INCLUDED
#define OFFB_CONTROLLER_INCLUDED

#include <string>

#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/VFR_HUD.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PointStamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/PositionTarget.h"
#include "opencv2/core.hpp"

#include "dt_config.hpp"
#include "offb_controller.hpp"
#include "offb_pid.hpp"
#include "offb_filter.hpp"

extern volatile int g_flight_exit;

class OffbController {
public:
  int init();
  void prepStandby();
  int prepFlight();
  void loop();  // Main flight loop

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
  int rp_stream_rate;

private:
  // Ros Comms
  ros::ServiceClient mode_client;
  ros::ServiceClient arm_client;
  ros::Subscriber stateSub;
  ros::Subscriber vfrSub;
  ros::Subscriber imuSub;
  ros::Subscriber camSub;
  ros::Publisher raw_pub;

  // Utilities
  template<typename T> void readParam(const std::string& param_name, T* var, const T& defaultVal);
  int setMode(std::string modeName);
  int waitForArm();
  void zeroAltitude();
  ros::Timer startTimeout(double duration);

  // PID controllers
  OffbPID altPID;
  OffbPID pitchPID;
  OffbPID rollPID;
  OffbPID yawPID;
  OffbPID ofPitchPID;
  OffbPID ofRollPID;
  OffbFilter ofPitchFilter;
  OffbFilter ofRollFilter;
  float lostYawDir;

  // Data storage
  mavros_msgs::State currentState;
  mavros_msgs::VFR_HUD flightData;
  mavros_msgs::PositionTarget camData;
  sensor_msgs::Imu imuData;
  mavros_msgs::AttitudeTarget rawAtt;
  double zeroAlt;
  bool timeout;

  // Ros parameters
  int rp_loop_rate;
  int rp_man_testing;
  float rp_lim_nav_alt, rp_lim_nav_vs;
  float rp_lim_abort_alt, rp_lim_abort_lost_time;
  float rp_pid_alt_target;
  float rp_pid_alt_out_min, rp_pid_alt_out_max;
  float rp_pid_alt_out_ramp, rp_pid_alt_out_bias;
  float rp_pid_alt_p, rp_pid_alt_i, rp_pid_alt_d;
  float rp_pid_of_pitch_target;
  float rp_pid_of_pitch_out_min, rp_pid_of_pitch_out_max, rp_pid_of_pitch_out_ramp;
  float rp_pid_of_pitch_p, rp_pid_of_pitch_d;
  float rp_of_pitch_ftconst;
  float rp_pid_of_roll_target;
  float rp_pid_of_roll_out_min, rp_pid_of_roll_out_max;
  float rp_pid_of_roll_out_ramp, rp_pid_roll_out_bias;
  float rp_pid_of_roll_p, rp_pid_of_roll_d;
  float rp_of_roll_ftconst;
  float rp_pid_pitch_target;
  float rp_pid_pitch_out_min, rp_pid_pitch_out_max;
  float rp_pid_pitch_out_ramp, rp_pid_pitch_out_bias;
  float rp_pid_pitch_p, rp_pid_pitch_d;
  float rp_pid_roll_target;
  float rp_pid_roll_out_min, rp_pid_roll_out_max, rp_pid_roll_out_ramp;
  float rp_pid_roll_p, rp_pid_roll_d;
  float rp_pid_yaw_target;
  float rp_pid_yaw_out_min, rp_pid_yaw_out_max, rp_pid_yaw_out_ramp;
  float rp_pid_yaw_p, rp_pid_yaw_d;
  float rp_roll_yaw_coupling, rp_lost_yaw_rate;

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
