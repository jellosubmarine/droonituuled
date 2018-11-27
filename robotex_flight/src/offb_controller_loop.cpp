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
    ROS_INFO("Starting flight loop");
  #endif

  mavros_msgs::AttitudeTarget rawAtt;
  rawAtt.type_mask =  rawAtt.IGNORE_ROLL_RATE |
                      rawAtt.IGNORE_PITCH_RATE;

  geometry_msgs::Point floorPoint;
  double yaw = 0.0, pitch = 0.0, roll = 0.0;
  double relAlt = 0.0;

  altPID.target = rp_pid_alt_target;
  pitchPID.target = rp_pid_pitch_target;
  rollPID.target = rp_pid_roll_target;
  yawPID.target = rp_pid_yaw_target;

  ros::Rate loopRate(OFFB_FLIGHT_LOOP_RATE);
  double t = ros::Time::now().toSec();
  double lostTime = -1;  // Time when nav got lost

  ros::spinOnce();
  altPID.initFirstInput(flightData.altitude - zeroAlt, t);

  int flightStatus =
    (flightData.altitude - zeroAlt >= OFFB_MIN_FLIGHT_ALT) *
    FLIGHT_STATUS_FLYING;


  ROS_INFO("Starting flight loop");
  while (!g_flight_exit && !ros::isShuttingDown() && currentState.mode != "LAND") {
    t = ros::Time::now().toSec();
    relAlt = flightData.altitude - zeroAlt;


    // Check abort altitude
    if (relAlt > rp_lim_abort_alt) {
      ROS_INFO("Exceeded ABORT ALT");
      SET_BIT(flightStatus, FLIGHT_STATUS_LAND);
    }

    // Check lost time
    if (GET_BIT(flightStatus, FLIGHT_STATUS_LOST) &&
        (t - lostTime > rp_lim_abort_lost_time))
    {
      ROS_INFO("Exceeded ABORT LOST TIME");
      SET_BIT(flightStatus, FLIGHT_STATUS_LAND);
    }

    // Check if mission abort is needed
    if (GET_BIT(flightStatus, FLIGHT_STATUS_LAND)) {
      setMode("LAND");
      continue;
    }


    // Always maintain altitude
    altPID.update(flightData.altitude - zeroAlt, t);
    rawAtt.thrust = altPID.output;


    // Drone on ground
    if (! GET_BIT(flightStatus, FLIGHT_STATUS_FLYING)) {
      rawAtt.body_rate.z = 0.0;
      rawAtt.orientation.w = 1.0;
      rawAtt.orientation.x = 0.0;
      rawAtt.orientation.y = 0.0;
      rawAtt.orientation.z = 0.0;

      if (relAlt >= OFFB_MIN_FLIGHT_ALT) {
        ROS_INFO("Airborne");
        SET_BIT(flightStatus, FLIGHT_STATUS_FLYING);
      }
    }


    // Drone in air
    else {
      // Check if drone has lost visuals
      if (camData.position.z <= 1 &&
          !GET_BIT(flightStatus, FLIGHT_STATUS_LOST)) {
        ROS_INFO("Set LOST status");
        SET_BIT(flightStatus, FLIGHT_STATUS_LOST);
        CLEAR_BIT(flightStatus, FLIGHT_STATUS_NAVIGATE);
        lostTime = ros::Time::now().toSec();
      }


      // IF LOST
      if (GET_BIT(flightStatus, FLIGHT_STATUS_LOST)) {
        // Look for any lines unless we're taking off
        if (fabs(flightData.climb) < rp_lim_nav_vs)
          rawAtt.body_rate.z = rp_lost_yaw_rate;
        else
          rawAtt.body_rate.z = 0.0;

        // Don't attempt to control movement
        rawAtt.orientation.w = 1.0;
        rawAtt.orientation.x = 0.0;
        rawAtt.orientation.y = 0.0;
        rawAtt.orientation.z = 0.0;

        if (camData.position.z) {
          ROS_INFO("Cleared LOST status");
          CLEAR_BIT(flightStatus, FLIGHT_STATUS_LOST);
          yawPID.initFirstInput(camData.yaw, t);
        }
      }


      // NOT LOST
      else {
        // Turn to correct heading
        yawPID.update(camData.yaw, t);
        rawAtt.body_rate.z = yawPID.output;

        // Convert camera data
        offb_quat2euler(&(imuData.orientation), &roll, &pitch, &yaw);
        offb_cam2floor(
          camData.position.x, camData.position.y, flightData.altitude,
          OFFB_CAM_ANGLE - pitch, OFFB_CAM_FOV_X, OFFB_CAM_FOV_Y,
          &floorPoint);

        // Navigate
        if (GET_BIT(flightStatus, FLIGHT_STATUS_NAVIGATE)) {
          pitchPID.update(floorPoint.y, camData.velocity.y, t);
          rollPID.update(floorPoint.x, camData.velocity.x, t);
          offb_euler2quat(rollPID.output + rawAtt.body_rate.z * rp_roll_yaw_coupling,
                          pitchPID.output, 0.0, &(rawAtt.orientation));

          // Check for altitude outside margins
          if (relAlt < rp_pid_alt_target - rp_lim_nav_alt ||
              relAlt > rp_pid_alt_target + rp_lim_nav_alt ||
              fabs(flightData.climb) > rp_lim_nav_vs )
          {
            ROS_INFO("Altitude unstable. Stopped navigation.");
            CLEAR_BIT(flightStatus, FLIGHT_STATUS_NAVIGATE);
          }
        }

        // Not safe to navigate (due to altitude)
        else {
          rawAtt.orientation.w = 1.0;
          rawAtt.orientation.x = 0.0;
          rawAtt.orientation.y = 0.0;
          rawAtt.orientation.z = 0.0;

          // Check if ready to navigate
          if (relAlt > rp_pid_alt_target - rp_lim_nav_alt &&
              relAlt < rp_pid_alt_target + rp_lim_nav_alt &&
              fabs(flightData.climb) < rp_lim_nav_vs )
          {
            ROS_INFO("Starting navigation");
            SET_BIT(flightStatus, FLIGHT_STATUS_NAVIGATE);
            pitchPID.initFirstInput(camData.position.y, t);
            rollPID.initFirstInput(camData.position.x, t);
          }
        }  // Navigate
      }  // lost
    }  // airborne


    debug(camData.yaw, floorPoint.y, floorPoint.x, 0.0f);
    raw_pub.publish(rawAtt);

    #ifdef OFFB_SHOW_VISUALS
      updateVisuals(floorPoint.x, floorPoint.y, camData.yaw);
    #endif

    loopRate.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Flight loop finished");
  setMode("LAND");
}

#endif
