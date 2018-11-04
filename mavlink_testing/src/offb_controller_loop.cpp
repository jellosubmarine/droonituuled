#include "offb_controller.hpp"
#include "offb_config.hpp"
#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/Quaternion.h"


void OffbController::loop() {
  ros::Rate loopRate(OFFB_FLIGHT_LOOP_RATE);
  mavros_msgs::AttitudeTarget rawAtt;
  double t = ros::Time::now().toSec();

  altPID.target = OFFB_ALT_TARGET;

  ROS_INFO_STREAM( "Time: " << ros::Time::now() << "  sec: " << t );
  geometry_msgs::Quaternion q;

  ros::spinOnce();
  altPID.initFirstInput(flightData.altitude - zeroAlt, t);

  ROS_INFO("Starting flight loop");
  while (ros::ok() && currentState.mode != "LAND") {
    t = ros::Time::now().toSec();

  	//rawAtt.thrust = 0.6;
  	//rawAtt.orientation.x = sin(20.0*M_PI/180.0/2.0) * cos(20.0*M_PI/180.0);
  	//rawAtt.orientation.w = cos(20.0*M_PI/180.0/2.0);
  	//rawAtt.type_mask = rawAtt.IGNORE_THRUST;

    altPID.update(flightData.altitude - zeroAlt, t);
    rawAtt.thrust = altPID.output;

    q.w = altPID.output;
    q.x = altPID.curErr;
    q.y = altPID.iOut;
    q.z = altPID.dOut;
    debug_pub.publish(q);

		raw_pub.publish(rawAtt);
		ros::spinOnce();
		loopRate.sleep();
  }

  ROS_INFO("Flight loop finished");
  setMode("LAND");
}
