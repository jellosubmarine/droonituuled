#include "offb_controller.hpp"
#include "offb_config.hpp"
#include "offb_math.hpp"

#include <cmath>
#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/Quaternion.h"

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
  mavros_msgs::AttitudeTarget rawAtt;
  rawAtt.type_mask =  rawAtt.IGNORE_ROLL_RATE |
                      rawAtt.IGNORE_PITCH_RATE;

  geometry_msgs::Point floorPoint;
  double yaw = 0.0, pitch = 0.0, roll = 0.0;

  altPID.target = OFFB_ALT_TARGET;
  pitchPID.target = OFFB_PITCH_TARGET;
  rollPID.target = OFFB_ROLL_TARGET;
  yawPID.target = OFFB_YAW_TARGET;

  //geometry_msgs::Quaternion q;

  ros::Rate loopRate(OFFB_FLIGHT_LOOP_RATE);
  double t = ros::Time::now().toSec();
  double lostTime = -1; // Time when nav got lost

  ros::spinOnce();
  altPID.initFirstInput(flightData.altitude - zeroAlt, t);
  //yawPID.initFirstInput(floorData.quaternion.z, t); // TODO: move to "NOT LOST" setting

  int flightStatus =
    (flightData.altitude - zeroAlt >= OFFB_MIN_FLIGHT_ALT) *
    FLIGHT_STATUS_FLYING;
  ROS_INFO_STREAM( "Flight status " << flightStatus );

/*
  mavros_msgs::AttitudeTarget zzzz;
  offb_euler2quat(5.0*M_PI/180.0, 5.0*M_PI/180.0, 0.0, &(zzzz.orientation));
  ROS_INFO_STREAM( "zzzz: " << zzzz );
*/

  ROS_INFO("Starting flight loop");
  while (!ros::isShuttingDown() && currentState.mode != "LAND") {
    t = ros::Time::now().toSec();
    double relAlt = flightData.altitude - zeroAlt;

    // Check abort altitude
    if (relAlt > OFFB_ABORT_ALT) {
      ROS_INFO("Exceeded ABORT ALT");
      SET_BIT(flightStatus, FLIGHT_STATUS_LAND);
    }

    // Check lost time
    if (GET_BIT(flightStatus, FLIGHT_STATUS_LOST) &&
        (t - lostTime > OFFB_ABORT_LOST_TIME))
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
      if (! floorData.quaternion.w &&
          ! GET_BIT(flightStatus, FLIGHT_STATUS_LOST)) {
          ROS_INFO("Set LOST status");
          SET_BIT(flightStatus, FLIGHT_STATUS_LOST);
          CLEAR_BIT(flightStatus, FLIGHT_STATUS_NAVIGATE);
          lostTime = ros::Time::now().toSec();
      }


      if (GET_BIT(flightStatus, FLIGHT_STATUS_LOST)) {
          rawAtt.body_rate.z = OFFB_LOST_YAW_RATE;
          rawAtt.orientation.w = 1.0;
          rawAtt.orientation.x = 0.0;
          rawAtt.orientation.y = 0.0;
          rawAtt.orientation.z = 0.0;

        if (floorData.quaternion.w) {
          ROS_INFO("Cleared LOST status");
          CLEAR_BIT(flightStatus, FLIGHT_STATUS_LOST);
          yawPID.initFirstInput(floorData.quaternion.z, t);
        }
      }

      // Visuals OK
      else {
        // Turn to correct heading
        yawPID.update(floorData.quaternion.z, t);
        rawAtt.body_rate.z = yawPID.output;

        // Convert camera data
        offb_quat2euler(&(imuData.orientation), &roll, &pitch, &yaw);
        offb_cam2floor(
          floorData.quaternion.x, floorData.quaternion.y, flightData.altitude,
          OFFB_CAM_ANGLE - pitch, OFFB_CAM_FOV_X, OFFB_CAM_FOV_Y,
          &floorPoint
        );

        if (GET_BIT(flightStatus, FLIGHT_STATUS_NAVIGATE)) {
          pitchPID.update(floorPoint.y, t);
          rollPID.update(floorPoint.x, t);
          offb_euler2quat(rollPID.output + rawAtt.body_rate.z * OFFB_ROLL_YAW_COUPL,
                          pitchPID.output, 0.0, &(rawAtt.orientation));

          // Check for altitude outside margins
          if (relAlt < OFFB_ALT_TARGET - OFFB_NAV_ALT_MARGIN ||
              relAlt > OFFB_ALT_TARGET + OFFB_NAV_ALT_MARGIN ||
              fabs(flightData.climb) > OFFB_NAV_MAX_CLIMB )
          {
            ROS_INFO("Altitude margin breach. Stopped navigation.");
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
          if (relAlt > OFFB_ALT_TARGET - OFFB_NAV_ALT_MARGIN &&
              relAlt < OFFB_ALT_TARGET + OFFB_NAV_ALT_MARGIN &&
              fabs(flightData.climb) < OFFB_NAV_MAX_CLIMB )
          {
            ROS_INFO("Starting navigation");
            SET_BIT(flightStatus, FLIGHT_STATUS_NAVIGATE);
            pitchPID.initFirstInput(floorData.quaternion.y, t);
            rollPID.initFirstInput(floorData.quaternion.x, t);
          }
        } // Navigate
      } // lost
    } // airborne

    //debug(rawAtt.orientation.w, rawAtt.orientation.x, rawAtt.orientation.y, rawAtt.orientation.z);
    debug(floorPoint.y, floorPoint.x, pitchPID.output*180.0/M_PI, rollPID.output*180.0/M_PI);
    //debug(floorData.quaternion.z*180.0/M_PI, yawPID.output*180.0/M_PI, -imuData.angular_velocity.z*180.0/M_PI, flightData.altitude);
    //debug(rawAtt.thrust, flightData.altitude, 0.0, 0.0);
		raw_pub.publish(rawAtt);
		ros::spinOnce();
		loopRate.sleep();
  }

  ROS_INFO("Flight loop finished");
  setMode("LAND");
}

#endif
