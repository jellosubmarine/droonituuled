#include <cmath>

#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/Quaternion.h"

#include "dt_config.hpp"
#include "offb_controller.hpp"
#include "offb_config.hpp"
#include "offb_math.hpp"

#ifdef OFFB_CONTROLLER_MODE_NAV

#define FLIGHT_STATUS_FLYING     1
#define FLIGHT_STATUS_LOST       2
#define FLIGHT_STATUS_LAND       4
#define FLIGHT_STATUS_NAVIGATE   8


// Publishes to the debug topic
void OffbController::debug(double w, double x, double y, double z) {
  debug_q.w = w;
  debug_q.x = x;
  debug_q.y = y;
  debug_q.z = z;
  debug_pub.publish(debug_q);
}


// Main flight loop
void OffbController::loop() {
  #ifdef OFFB_VERBOSE
    ROS_INFO("Offb loop");
  #endif

  // Init controller loop parameters
  rawAtt.body_rate.x = 0.0f;
  rawAtt.body_rate.y = 0.0f;
  rawAtt.type_mask = rawAtt.IGNORE_ROLL_RATE |
                     rawAtt.IGNORE_PITCH_RATE;
  rawAtt.header.frame_id = "0";

  geometry_msgs::Point floorPoint;
  double yaw = 0.0, pitch = 0.0, roll = 0.0;
  double pitchOut = 0.0, rollOut = 0.0;
  double relAlt = 0.0;

  altPID.target = rp_pid_alt_target;
  pitchPID.target = rp_pid_pitch_target;
  rollPID.target = rp_pid_roll_target;
  yawPID.target = rp_pid_yaw_target;
  ofPitchPID.target = rp_pid_of_pitch_target;
  ofRollPID.target = rp_pid_of_roll_target;

  ros::Rate loopRate(rp_loop_rate);
  ros::Time t = ros::Time::now();
  double tsec = t.toSec();
  double lostTime = tsec;  // Time when nav got lost
  lostYawDir = 1.0;

  // Update all callbacks
  ros::spinOnce();
  relAlt = flightData.altitude - zeroAlt;
  altPID.initFirstInput(relAlt, tsec);

  int flightStatus = FLIGHT_STATUS_LOST |
    ((relAlt >= OFFB_MIN_FLIGHT_ALT) * FLIGHT_STATUS_FLYING);


  // Main flight loop
  ROS_INFO("Starting flight loop");
  while (!g_flight_exit && !ros::isShuttingDown() &&
         currentState.mode != "LAND" &&
         (rp_man_testing || currentState.armed)) {
    ros::spinOnce();
    t = ros::Time::now();
    tsec = t.toSec();
    relAlt = flightData.altitude - zeroAlt;
    pitchOut = 0.0;
    rollOut = 0.0;

    // Check abort altitude
    if (relAlt > rp_lim_abort_alt) {
      ROS_INFO("Exceeded ABORT ALT");
      SET_BIT(flightStatus, FLIGHT_STATUS_LAND);
    }

    // Check lost time
    if (GET_BIT(flightStatus, FLIGHT_STATUS_LOST) &&
        (tsec - lostTime > rp_lim_abort_lost_time)) {
      ROS_INFO("Exceeded ABORT LOST TIME");
      SET_BIT(flightStatus, FLIGHT_STATUS_LAND);
    }

    // Check if mission abort is needed
    if (GET_BIT(flightStatus, FLIGHT_STATUS_LAND)) {
      setMode("LAND");
      continue;
    }


    // Always maintain altitude
    altPID.update(relAlt, tsec);
    rawAtt.thrust = altPID.output;


    // Drone on ground
    if (!GET_BIT(flightStatus, FLIGHT_STATUS_FLYING)) {
      rawAtt.body_rate.z = 0.0;
      rawAtt.orientation.w = 1.0;
      rawAtt.orientation.x = 0.0;
      rawAtt.orientation.y = 0.0;
      rawAtt.orientation.z = 0.0;

      if (relAlt >= OFFB_MIN_FLIGHT_ALT) {
        ROS_INFO("Airborne");
        SET_BIT(flightStatus, FLIGHT_STATUS_FLYING);
        ofPitchFilter.initFirstInput(0.0f, tsec);
        ofPitchPID.initFirstInput(ofPitchFilter.filter(camData.velocity.y, tsec), tsec);
        ofRollFilter.initFirstInput(0.0f, tsec);
        ofRollPID.initFirstInput(ofRollFilter.filter(camData.velocity.x, tsec), tsec);
        lostTime = tsec;
      }
    }


    // Drone in air
    else {
      // Check if drone has lost visuals
      if (!camData.position.z &&
          !GET_BIT(flightStatus, FLIGHT_STATUS_LOST)) {
        ROS_INFO("Set LOST status");
        SET_BIT(flightStatus, FLIGHT_STATUS_LOST);
        CLEAR_BIT(flightStatus, FLIGHT_STATUS_NAVIGATE);
        lostTime = tsec;
      }

      // TODO(johan): Combine redundant if statements
      // If not climing, do optical stabilisation
      if (fabs(flightData.climb) < rp_lim_nav_vs) {
        ofPitchPID.update(ofPitchFilter.filter(camData.velocity.y, tsec), tsec);
        ofRollPID.update(ofRollFilter.filter(camData.velocity.x, tsec), tsec);
        pitchOut += ofPitchPID.output;
        rollOut += ofRollPID.output;
      }

      // IF LOST
      if (GET_BIT(flightStatus, FLIGHT_STATUS_LOST)) {
        if (fabs(flightData.climb) < rp_lim_nav_vs) {
          // Look for any lines unless we're taking off
          rawAtt.body_rate.z = rp_lost_yaw_rate * lostYawDir;
        } else {
          // Otherwise do nothing
          rawAtt.body_rate.z = 0.0;
          pitchOut = 0.0;
          rollOut = 0.0;
        }

        // Check if drone has regained visuals
        if (camData.position.z) {
          ROS_INFO("Cleared LOST status");
          CLEAR_BIT(flightStatus, FLIGHT_STATUS_LOST);
          yawPID.initFirstInput(camData.yaw, tsec);
        }
      }


      // NOT LOST
      else {
        // Turn to correct heading
        yawPID.update(camData.yaw, tsec);
        rawAtt.body_rate.z = yawPID.output;

        // Convert camera data
        offb_quat2euler(&(imuData.orientation), &roll, &pitch, &yaw);
        offb_cam2floor(
          camData.position.x, camData.position.y, flightData.altitude,
          OFFB_CAM_ANGLE - pitch, OFFB_CAM_FOV_X, OFFB_CAM_FOV_Y,
          &floorPoint);
        lostYawDir = SGNF(floorPoint.x);

        // Navigate
        if (GET_BIT(flightStatus, FLIGHT_STATUS_NAVIGATE)) {
          pitchPID.update(floorPoint.y, ofPitchFilter.output, tsec);
          rollPID.update(floorPoint.x, ofRollFilter.output, tsec);
          pitchOut += pitchPID.output;
          rollOut += rollPID.output + rawAtt.body_rate.z * rp_roll_yaw_coupling;

          // Check for altitude outside margins
          if (relAlt < rp_pid_alt_target - rp_lim_nav_alt ||
              relAlt > rp_pid_alt_target + rp_lim_nav_alt ||
              fabs(flightData.climb) > rp_lim_nav_vs ) {
            ROS_INFO("Altitude unstable. Stopped navigation.");
            CLEAR_BIT(flightStatus, FLIGHT_STATUS_NAVIGATE);
          }
        }

        // Not safe to navigate (due to altitude)
        else {
          // Check if ready to navigate
          if (relAlt > rp_pid_alt_target - rp_lim_nav_alt &&
              relAlt < rp_pid_alt_target + rp_lim_nav_alt &&
              fabs(flightData.climb) < rp_lim_nav_vs ) {
            ROS_INFO("Starting navigation");
            SET_BIT(flightStatus, FLIGHT_STATUS_NAVIGATE);
            pitchPID.initFirstInput(floorPoint.y, tsec);
            rollPID.initFirstInput(floorPoint.x, tsec);
          }
        }  // Navigate
      }  // lost
    }  // airborne

   // Output and debug
    offb_euler2quat(rollOut, pitchOut, 0.0, &(rawAtt.orientation));
    rawAtt.header.stamp = t;
    raw_pub.publish(rawAtt);

    // debug(0.0, roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI);
    // debug(floorPoint.y, camData.position.x, camData.position.y, floorPoint.x);
    // debug(rollPID.output, rollPID.iOut, rollPID.dOut, rollPID.dState);
    // debug(0.0, rollOut*180.0/M_PI, pitchOut*180.0/M_PI, 0.0);
    debug(ofPitchFilter.output, camData.velocity.x, camData.velocity.y, ofRollFilter.output);
    #ifdef OFFB_SHOW_VISUALS
      updateVisuals(floorPoint.x, floorPoint.y, camData.yaw);
    #endif

    loopRate.sleep();
  }

  ROS_INFO("Flight loop finished");
  setMode("LAND");
}

#endif
