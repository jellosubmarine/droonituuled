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
  // Set up comms
  ROS_INFO("Setting up ROS comms");
  arm_client = nh.serviceClient<mavros_msgs::CommandBool> ("/mavros/cmd/arming");
  mode_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");

  stateSub = nh.subscribe<mavros_msgs::State> ("/mavros/state", 3, &OffbController::stateCB, this);
  vfrSub = nh.subscribe<mavros_msgs::VFR_HUD> ("/mavros/vfr_hud", 3, &OffbController::vfrCB, this);
  imuSub = nh.subscribe<sensor_msgs::Imu> ("/mavros/imu/data", 3, &OffbController::imuCB, this);
  floorSub = nh.subscribe<geometry_msgs::QuaternionStamped> (DT_CAM_TOPIC, 3, &OffbController::floorCB, this);

  raw_pub = nh.advertise<mavros_msgs::AttitudeTarget> ("/mavros/setpoint_raw/attitude", 3, true);
  debug_pub = nh.advertise<geometry_msgs::Quaternion> (DT_DEBUG_TOPIC, 3, false);

  // Configure PID-s
  ROS_INFO("Configuring PIDs");
  altPID.conf(
    OFFB_PID_ALT_P, OFFB_PID_ALT_I, OFFB_PID_ALT_D, OFFB_PID_ALT_F,
    OFFB_PID_ALT_DTC, OFFB_PID_ALT_DK, OFFB_PID_ALT_BIAS,
    OFFB_PID_ALT_MAX_OUTPUT, OFFB_PID_ALT_MIN_OUTPUT,
    OFFB_PID_ALT_MAX_OUTPUT_RAMP
  );

  pitchPID.conf(
    OFFB_PID_PITCH_P, OFFB_PID_PITCH_I, OFFB_PID_PITCH_D, OFFB_PID_PITCH_F,
    OFFB_PID_PITCH_DTC, OFFB_PID_PITCH_DK, OFFB_PID_PITCH_BIAS,
    OFFB_PID_PITCH_MAX_OUTPUT, OFFB_PID_PITCH_MIN_OUTPUT,
    OFFB_PID_PITCH_MAX_OUTPUT_RAMP
  );

  rollPID.conf(
    OFFB_PID_ROLL_P, OFFB_PID_ROLL_I, OFFB_PID_ROLL_D, OFFB_PID_ROLL_F,
    OFFB_PID_ROLL_DTC, OFFB_PID_ROLL_DK, OFFB_PID_ROLL_BIAS,
    OFFB_PID_ROLL_MAX_OUTPUT, OFFB_PID_ROLL_MIN_OUTPUT,
    OFFB_PID_ROLL_MAX_OUTPUT_RAMP
  );

  yawPID.conf(
    OFFB_PID_YAW_P, OFFB_PID_YAW_I, OFFB_PID_YAW_D, OFFB_PID_YAW_F,
    OFFB_PID_YAW_DTC, OFFB_PID_YAW_DK, OFFB_PID_YAW_BIAS,
    OFFB_PID_YAW_MAX_OUTPUT, OFFB_PID_YAW_MIN_OUTPUT,
    OFFB_PID_YAW_MAX_OUTPUT_RAMP
  );


  //ROS_INFO("REMOVE DELAY FROM CTRL INIT");
  //ros::Duration(OFFB_DEBUG_START_DELAY).sleep();
  #warning "Remove commented delay code"

	// Connect to FCU
	ROS_INFO("Waiting for FCU connection");
  ros::Rate loopRate(OFFB_START_LOOP_RATE);

  ros::Timer t = startTimeout(OFFB_GEN_TIMEOUT);
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

/**
 * Stops the flight loop and shuts down ros
 */
void OffbController::shutdown() {
  //setMode("LAND");
  ros::shutdown();
}

/** CALLBACKS **/

// State update
void OffbController::stateCB (const mavros_msgs::State::ConstPtr& msg) {
  	currentState = *msg;
}

void OffbController::vfrCB (const mavros_msgs::VFR_HUD::ConstPtr& msg) {
  flightData = *msg;
}

void OffbController::imuCB (const sensor_msgs::Imu::ConstPtr& msg) {
  imuData = *msg;
}

void OffbController::floorCB (const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
  floorData = *msg;
}

void OffbController::timeoutCB (const ros::TimerEvent&){
  ROS_INFO("Timeout triggered");
  timeout = true;
}



/** UTILITIES **/

/**
 * Creates and starts a timeout timer
 * NB! Must not start several timout timers at the same time,
 * there's only one timout variable
 */
ros::Timer OffbController::startTimeout(double duration) {
  timeout = false;
  return nh.createTimer(
            ros::Duration(duration),
            &OffbController::timeoutCB,
            this, true
  );
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

	ros::Timer t = startTimeout(OFFB_GEN_TIMEOUT);
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
  #ifndef OFFB_WAIT_FOR_ARM
    ROS_INFO("Arming vehicle");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (!arm_client.call(arm_cmd)) {
      ROS_INFO("Arm call failed");
      return 0;
    };
  #else
    ROS_INFO("Waiting for vehicle to be armed");
  #endif

  ros::Rate delay(OFFB_START_LOOP_RATE);
  ros::Timer t = startTimeout(OFFB_ARM_TIMEOUT);
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

  ROS_INFO("Vehicle not armed");
  return 0;
}
