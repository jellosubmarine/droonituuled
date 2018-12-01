#include "dt_config.hpp"
#include "offb_controller.hpp"
#include "offb_config.hpp"

#include "ros/ros.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/AttitudeTarget.h"


/**
 * Set up mavros communication, connect to FCU, arm vehicle and zero altitude
 */
int OffbController::init() {
  #ifdef OFFB_VERBOSE
    ROS_INFO("Initialising flight controller");
  #endif

  // Load ros parameters
  readParam("loop_rate", &rp_loop_rate, 10);
  readParam("manual_testing", &rp_man_testing, 0);
  readParam("limit/abort/altitude", &rp_lim_abort_alt, 2.0f);
  readParam("limit/abort/lost_time", &rp_lim_abort_lost_time, 10.0f);
  readParam("limit/nav/alt_margin", &rp_lim_nav_alt, 0.3f);
  readParam("limit/nav/max_vs", &rp_lim_nav_vs, 0.3f);
  readParam("pid/altitude/target", &rp_pid_alt_target, 1.5f);
  readParam("pid/altitude/output/min", &rp_pid_alt_out_min, 0.0f);
  readParam("pid/altitude/output/max", &rp_pid_alt_out_max, 1.0f);
  readParam("pid/altitude/output/ramp", &rp_pid_alt_out_ramp, 1.0f);
  readParam("pid/altitude/output/bias", &rp_pid_alt_out_bias, 0.0f);
  readParam("pid/altitude/p", &rp_pid_alt_p, 0.0f);
  readParam("pid/altitude/i", &rp_pid_alt_i, 0.0f);
  readParam("pid/altitude/d", &rp_pid_alt_d, 0.0f);
  readParam("pid/of_pitch/target", &rp_pid_of_pitch_target, 1.0f);
  readParam("pid/of_pitch/output/min", &rp_pid_of_pitch_out_min, -1.0f);
  readParam("pid/of_pitch/output/max", &rp_pid_of_pitch_out_max, +1.0f);
  readParam("pid/of_pitch/output/ramp", &rp_pid_of_pitch_out_ramp, 1.0f);
  readParam("pid/of_pitch/p", &rp_pid_of_pitch_p, 0.0f);
  readParam("pid/of_pitch/d", &rp_pid_of_pitch_d, 0.0f);
  readParam("pid/of_pitch/filter_tconst", &rp_of_pitch_ftconst, 1.0f);
  readParam("pid/of_roll/target", &rp_pid_of_roll_target, 1.0f);
  readParam("pid/of_roll/output/min", &rp_pid_of_roll_out_min, -1.0f);
  readParam("pid/of_roll/output/max", &rp_pid_of_roll_out_max, +1.0f);
  readParam("pid/of_roll/output/ramp", &rp_pid_of_roll_out_ramp, 1.0f);
  readParam("pid/of_roll/p", &rp_pid_of_roll_p, 0.0f);
  readParam("pid/of_roll/d", &rp_pid_of_roll_d, 0.0f);
  readParam("pid/of_roll/filter_tconst", &rp_of_roll_ftconst, 1.0f);
  readParam("pid/pitch/target", &rp_pid_pitch_target, 1.0f);
  readParam("pid/pitch/output/min", &rp_pid_pitch_out_min, -1.0f);
  readParam("pid/pitch/output/max", &rp_pid_pitch_out_max, +1.0f);
  readParam("pid/pitch/output/ramp", &rp_pid_pitch_out_ramp, 1.0f);
  readParam("pid/pitch/p", &rp_pid_pitch_p, 0.0f);
  readParam("pid/pitch/d", &rp_pid_pitch_d, 0.0f);
  readParam("pid/roll/target", &rp_pid_roll_target, 1.0f);
  readParam("pid/roll/output/min", &rp_pid_roll_out_min, -1.0f);
  readParam("pid/roll/output/max", &rp_pid_roll_out_max, +1.0f);
  readParam("pid/roll/output/ramp", &rp_pid_roll_out_ramp, 1.0f);
  readParam("pid/roll/p", &rp_pid_roll_p, 0.0f);
  readParam("pid/roll/d", &rp_pid_roll_d, 0.0f);
  readParam("pid/yaw/target", &rp_pid_yaw_target, 1.0f);
  readParam("pid/yaw/output/min", &rp_pid_yaw_out_min, -1.0f);
  readParam("pid/yaw/output/max", &rp_pid_yaw_out_max, +1.0f);
  readParam("pid/yaw/output/ramp", &rp_pid_yaw_out_ramp, 1.0f);
  readParam("pid/yaw/p", &rp_pid_yaw_p, 0.0f);
  readParam("pid/yaw/d", &rp_pid_yaw_d, 0.0f);
  readParam("pid/roll_yaw_coupling", &rp_roll_yaw_coupling, 0.0f);
  readParam("pid/lost_yaw_rate", &rp_lost_yaw_rate, 30.0f);
  // readParam("stream/id", &stream_id, 0);
  readParam("stream/rate", &rp_stream_rate, 10);
  // readParam("stream/on_off", &stream_on_off, true);

  // Set up comms
  #ifdef OFFB_VERBOSE
    ROS_INFO("Setting up ROS comms");
  #endif
  arm_client = nh.serviceClient<mavros_msgs::CommandBool> ("/mavros/cmd/arming");
  mode_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");

  stateSub = nh.subscribe<mavros_msgs::State> ("/mavros/state", 3, &OffbController::stateCB, this);
  vfrSub = nh.subscribe<mavros_msgs::VFR_HUD> ("/mavros/vfr_hud", 3, &OffbController::vfrCB, this);
  imuSub = nh.subscribe<sensor_msgs::Imu> ("/mavros/imu/data", 3, &OffbController::imuCB, this);
  camSub = nh.subscribe<mavros_msgs::PositionTarget> (DT_CAM_TOPIC, 3, &OffbController::camCB, this);

  raw_pub = nh.advertise<mavros_msgs::AttitudeTarget> ("/mavros/setpoint_raw/attitude", 3, true);
  debug_pub = nh.advertise<geometry_msgs::Quaternion> (DT_DEBUG_TOPIC, 3, false);

  // Configure PID-s
  #ifdef OFFB_VERBOSE
    ROS_INFO("Configuring PIDs");
  #endif
  altPID.conf(
    rp_pid_alt_p, rp_pid_alt_i, rp_pid_alt_d, OFFB_PID_ALT_F,
    OFFB_PID_ALT_DTC, OFFB_PID_ALT_DK, rp_pid_alt_out_bias,
    rp_pid_alt_out_max, rp_pid_alt_out_min,
    rp_pid_alt_out_ramp);

  pitchPID.conf(
    rp_pid_pitch_p, OFFB_PID_PITCH_I, rp_pid_pitch_d, OFFB_PID_PITCH_F,
    OFFB_PID_PITCH_DTC, OFFB_PID_PITCH_DK, OFFB_PID_PITCH_BIAS,
    rp_pid_pitch_out_max, rp_pid_pitch_out_min,
    rp_pid_pitch_out_ramp);

  rollPID.conf(
    rp_pid_roll_p, OFFB_PID_ROLL_I, rp_pid_roll_d, OFFB_PID_ROLL_F,
    OFFB_PID_ROLL_DTC, OFFB_PID_ROLL_DK, OFFB_PID_ROLL_BIAS,
    rp_pid_roll_out_max, rp_pid_roll_out_min,
    rp_pid_roll_out_ramp);

  yawPID.conf(
    rp_pid_yaw_p, OFFB_PID_YAW_I, rp_pid_yaw_d, OFFB_PID_YAW_F,
    OFFB_PID_YAW_DTC, OFFB_PID_YAW_DK, OFFB_PID_YAW_BIAS,
    rp_pid_yaw_out_max, rp_pid_yaw_out_min,
    rp_pid_yaw_out_ramp);

  ofRollPID.conf(
    rp_pid_of_roll_p, OFFB_PID_ROLL_I, rp_pid_of_roll_d, OFFB_PID_ROLL_F,
    OFFB_PID_ROLL_DTC, OFFB_PID_ROLL_DK, OFFB_PID_ROLL_BIAS,
    rp_pid_of_roll_out_max, rp_pid_of_roll_out_min,
    rp_pid_of_roll_out_ramp);

  ofPitchPID.conf(
    rp_pid_of_pitch_p, OFFB_PID_PITCH_I, rp_pid_of_pitch_d, OFFB_PID_PITCH_F,
    OFFB_PID_PITCH_DTC, OFFB_PID_PITCH_DK, OFFB_PID_PITCH_BIAS,
    rp_pid_of_pitch_out_max, rp_pid_of_pitch_out_min,
    rp_pid_of_pitch_out_ramp);

  ofRollFilter.conf(rp_of_roll_ftconst);
  ofPitchFilter.conf(rp_of_pitch_ftconst);

  // Init visuals
  #ifdef OFFB_SHOW_VISUALS
    initVisuals();
  #endif

  // Connect to FCU
  #ifdef OFFB_VERBOSE
    ROS_INFO("Waiting for FCU connection");
  #endif
  ros::Rate loopRate(OFFB_START_LOOP_RATE);

  ros::Timer t = startTimeout(OFFB_GEN_TIMEOUT);
  while (!g_flight_exit &&
         ros::ok() &&
         !currentState.connected &&
         !timeout) {
    ros::spinOnce();
    loopRate.sleep();
  }
  t.stop();

  if (ros::ok() && currentState.connected) {
    ROS_INFO("Connected to FCU");
    return 1;
  }

  return 0;
}

// Waits for the motors to be disarmed before next flight
void OffbController::prepStandby() {
  ROS_INFO("Waiting for DISARM");
  ros::Rate loopRate(OFFB_START_LOOP_RATE);

  while (!g_flight_exit && currentState.armed) {
    ros::spinOnce();
    loopRate.sleep();
  }
}

// Preps the drone and controller for flight
int OffbController::prepFlight() {
    if (!setMode("STABILIZE")) return 0;     // Enter stabilize mode
    if (!rp_man_testing) {
      if (!waitForArm())
        return 0;             // Arm vehice
    } else {
      ROS_INFO("Skipping ARM status");
    }
    if (!setMode("GUIDED_NOGPS")) return 0;  // Set guided_nogps mode
    zeroAltitude();
    return 1;
}


/** CALLBACKS **/

// State update
void OffbController::stateCB(const mavros_msgs::State::ConstPtr& msg) {
  currentState = *msg;
}

void OffbController::vfrCB(const mavros_msgs::VFR_HUD::ConstPtr& msg) {
  flightData = *msg;
}

void OffbController::imuCB(const sensor_msgs::Imu::ConstPtr& msg) {
  imuData = *msg;
}

void OffbController::camCB(const mavros_msgs::PositionTarget::ConstPtr& msg) {
  camData = *msg;
}

void OffbController::timeoutCB(const ros::TimerEvent&) {
  #ifdef OFFB_VERBOSE
    ROS_INFO("Timeout triggered");
  #endif
  timeout = true;
}



/** UTILITIES **/

/**
 * Reads ros parameters from conf file
 */
template<typename T>
void OffbController::readParam(const std::string& param_name, T* var, const T& defaultVal) {
  if (nh.param(param_name, *var, defaultVal)) {
    ROS_INFO_STREAM("Got param: " << param_name << " = " << *var);
  } else {
    ROS_ERROR_STREAM("Failed to get param " << param_name << ", default = " << defaultVal);
  }
}

/**
 * Creates and starts a timeout timer
 * NB! Must not start several timout timers at the same time,
 * there's only one timout variable
 */
ros::Timer OffbController::startTimeout(double duration) {
  timeout = false;
  return nh.createTimer(ros::Duration(duration),
                        &OffbController::timeoutCB,
                        this, true);
}

/**
 * Finds current mean altitude and sets it to altitude offset
 */
void OffbController::zeroAltitude() {
  int sampleCounter = 0;
  float altSum = 0;
  #ifdef OFFB_VERBOSE
    ROS_INFO("Determining altitude offset");
  #endif

  ros::Rate loopRate(OFFB_START_LOOP_RATE);
  ros::Timer t = startTimeout(OFFB_ZEROALT_TIME);
  while (!g_flight_exit && ros::ok() && !timeout) {
    ros::spinOnce();
    altSum += flightData.altitude;
    sampleCounter++;
    loopRate.sleep();
  }
  t.stop();

  zeroAlt = altSum / float(sampleCounter);
  ROS_INFO_STREAM("Altitude offset: " << zeroAlt);
}

/**
 * Sets ardupilot to the requested mode
 * Returns 1 on success, 0 on failure
 */
int OffbController::setMode(std::string modeName) {
	ROS_INFO_STREAM("Setting " << modeName << " mode");
	mavros_msgs::SetMode modeCmd;
	modeCmd.request.custom_mode = modeName;

	ros::Rate delay(OFFB_START_LOOP_RATE);

	if (!mode_client.call(modeCmd)) {
		ROS_INFO("Mode call failed");
		return 0;
	};

  ros::Timer t = startTimeout(OFFB_GEN_TIMEOUT);
  while (!g_flight_exit &&
         ros::ok() &&
         currentState.mode != modeName &&
         !timeout) {
    ros::spinOnce();
    delay.sleep();
  }
  t.stop();

	if (ros::ok() && currentState.mode == modeName) {
		ROS_INFO_STREAM("Mode " << modeName << " set");
		return 1;
	}

	ROS_INFO_STREAM( "Failed to set mode " << modeName );
	return 0;
}


/**
 * Waits for the vehicle to be armed
 * Returns 1 on success, 0 on failure
 */
int OffbController::waitForArm() {
  ROS_INFO("Waiting for ARM");
  ros::Rate loopRate(OFFB_START_LOOP_RATE);

  while (!g_flight_exit &&
         !ros::isShuttingDown() &&
         !currentState.armed &&
         currentState.mode != "LAND") {
    loopRate.sleep();
    ros::spinOnce();
  }

  if (!g_flight_exit && ros::ok() && currentState.armed) {
    #ifdef OFFB_VERBOSE
      ROS_INFO("Vehicle armed");
    #endif
    return 1;
  }

  ROS_INFO("Vehicle not armed");
  return 0;
}
