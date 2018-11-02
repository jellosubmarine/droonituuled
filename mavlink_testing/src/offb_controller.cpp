#include "offb_controller.hpp"
#include "offb_config.hpp"

#include "ros/ros.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/Quaternion.h"


/**
 * Set up mavros communication, connect to FCU, arm vehicle and zero altitude
 */
int OffbController::init() {
  // Set up comms
  ROS_INFO("Setting up ROS comms");
  arm_client = nh.serviceClient<mavros_msgs::CommandBool> ("/mavros/cmd/arming");
  mode_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");

  state_sub = nh.subscribe<mavros_msgs::State> ("/mavros/state", 3, &OffbController::state_cb, this);
  vfr_sub = nh.subscribe<mavros_msgs::VFR_HUD> ("/mavros/vfr_hud", 3, &OffbController::vfr_cb, this);

  raw_pub = nh.advertise<mavros_msgs::AttitudeTarget> ("/mavros/setpoint_raw/attitude", 3, true);
  debug_pub = nh.advertise<geometry_msgs::Quaternion> ("/jk/debug", 3, false);

  // Configure PID-s
  ROS_INFO("Configuring PIDs");
  altPID.conf( OFFB_PID_ALT_P, OFFB_PID_ALT_I, OFFB_PID_ALT_D, OFFB_PID_ALT_F,
    OFFB_PID_ALT_DTC, OFFB_PID_ALT_DK, OFFB_PID_ALT_MAX_OUTPUT,
    OFFB_PID_ALT_MIN_OUTPUT, OFFB_PID_ALT_MAX_OUTPUT_RAMP
  );

  ros::Duration(OFFB_DEBUG_START_DELAY).sleep();
  // TODO: REMOVE THIS

	// Connect to FCU
	ROS_INFO("Waiting for FCU connection");
  ros::Rate loopRate(OFFB_START_LOOP_RATE);

  ros::Timer t = startTimeout(OFFB_TIMEOUT);
	while (ros::ok() && !currentState.connected && !timeout) {
		ros::spinOnce();
		loopRate.sleep();
	}
  t.stop();

	if (ros::ok() && currentState.connected) ROS_INFO("Connected to FCU");

	if (!setMode("STABILIZE")) return 0;     // Enter stabilize mode
	if (!armVehicle()) return 0;             // Arm vehice
	if (!setMode("GUIDED_NOGPS")) return 0;  // Set guided_nogps mode

  zeroAltitude();
  return 1;
}


/** CALLBACKS **/

// State update
void OffbController::state_cb(const mavros_msgs::State::ConstPtr& msg) {
  	currentState = *msg;
}

void OffbController::vfr_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg) {
  flightData = *msg;
}

void OffbController::timeout_cb(const ros::TimerEvent&){
  ROS_INFO("Timeout triggered");
  timeout = true;
}



/** UTILITIES **/

/**
 * Creates and starts a timeout timer
 */
ros::Timer OffbController::startTimeout(double duration) {
  timeout = false;
  return nh.createTimer(ros::Duration(duration), &OffbController::timeout_cb, this, true);
}

/**
 * Finds current mean altitude and sets it to altitude offset
 */
void OffbController::zeroAltitude() {
  int sampleCounter = 0;
  double altSum = 0;
  ROS_INFO("Determining altitude offset");

  ros::Rate loopRate(OFFB_START_LOOP_RATE);
  ros::Timer t = startTimeout(OFFB_ZEROALT_TIME);
  while(ros::ok() && !timeout) {
    altSum += flightData.altitude;
    sampleCounter++;
    ros::spinOnce();
    loopRate.sleep();
  }
  t.stop();

  zeroAlt = altSum / double(sampleCounter);
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

	ros::Timer t = startTimeout(OFFB_TIMEOUT);
	while ( ros::ok() &&
	 				currentState.mode != modeName &&
				 	! timeout )
	{
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
 * Attempts to arm the vehicle
 * Returns 1 on success, 0 on failure
 */
int OffbController::armVehicle() {
  ROS_INFO("Arming vehicle");
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

	ros::Rate delay(OFFB_START_LOOP_RATE);

  if (!arm_client.call(arm_cmd)) {
    ROS_INFO("Arm call failed");
    return 0;
  };

  ros::Timer t = startTimeout(OFFB_TIMEOUT);
  while ( ros::ok() &&
          !currentState.armed &&
          currentState.mode != "LAND" &&
          ! timeout )
  {
    ros::spinOnce();
    delay.sleep();
  }
  t.stop();

  if (ros::ok() && currentState.armed) {
    ROS_INFO("Vehicle armed");
    return 1;
  }

  ROS_INFO("Failed to arm vehicle");
  return 0;
}
