#include "offb_controller.hpp"
#include "offb_config.hpp"

#include "ros/ros.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"


/** CALLBACKS **/

// State update
void OffbController::state_cb(const mavros_msgs::State::ConstPtr& msg) {
  	current_state = *msg;
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
 * Set up mavros communication
 */
int OffbController::setup() {
  state_sub = this->nh.subscribe<mavros_msgs::State> ("/mavros/state", 3, &OffbController::state_cb, this);
  arm_client = nh.serviceClient<mavros_msgs::CommandBool> ("/mavros/cmd/arming");
  mode_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
  return 1;
}

/**
 * Sets ardupilot to the requested mode
 * Returns 1 on success, 0 on failure
 */
int OffbController::setMode(std::string mode_name) {
	ROS_INFO_STREAM("Setting " << mode_name << " mode");
	mavros_msgs::SetMode mode_cmd;
	mode_cmd.request.custom_mode = mode_name;

	ros::Rate delay(OFFB_LOOP_RATE);

	if (!mode_client.call(mode_cmd)) {
		ROS_INFO("Mode call failed");
		return 0;
	};

	ros::Timer t = startTimeout(OFFB_TIMEOUT);
	while ( ros::ok() &&
	 				current_state.mode != mode_name &&
				 	! timeout )
	{
		ros::spinOnce();
		delay.sleep();
	}
  t.stop();

	if (ros::ok() && current_state.mode == mode_name) {
		ROS_INFO_STREAM("Mode " << mode_name << " set");
		return 1;
	}

	ROS_INFO_STREAM( "Failed to set mode " << mode_name );
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

	ros::Rate delay(OFFB_LOOP_RATE);

  if (!arm_client.call(arm_cmd)) {
    ROS_INFO("Arm call failed");
    return 0;
  };

  ros::Timer t = startTimeout(OFFB_TIMEOUT);
  while ( ros::ok() &&
          !current_state.armed &&
          current_state.mode != "LAND" &&
          ! timeout )
  {
    ros::spinOnce();
    delay.sleep();
  }
  t.stop();

  if (ros::ok() && current_state.armed) {
    ROS_INFO("Vehicle armed");
    return 1;
  }

  ROS_INFO("Failed to arm vehicle");
  return 0;
}
