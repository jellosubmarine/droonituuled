#include <cmath>

#include "ros/ros.h"
#include "offb_math.hpp"



// Simplified projection transformation
// Disregards roll angle
// Normalised x and y from frame centre, zero angle = down
// FOV = from centre to edge
void offb_cam2floor( double x, double y, double alt,
                     double cam_pitch, double cam_fov_x, double cam_fov_y,
                     geometry_msgs::Point *ret ) {
  //geometry_msgs::Point p;
  ret->y = tan(cam_pitch + y * cam_fov_y) * alt;
  ret->x = sqrt(ret->y * ret->y + alt * alt) * tan(x * cam_fov_x);
}

// Converts yaw-pitch-roll to unit quaternion
// Angles in radians
void offb_euler2quat(double roll, double pitch, double yaw, geometry_msgs::Quaternion *ret) {
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    ret->w = cy * cr * cp + sy * sr * sp;
    ret->x = cy * sr * cp - sy * cr * sp;
    ret->y = cy * cr * sp + sy * sr * cp;
    ret->z = sy * cr * cp - cy * sr * sp;
}

void offb_quat2euler(const geometry_msgs::Quaternion *q,
                     double *roll, double *pitch, double *yaw ) {
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q->w * q->x + q->y * q->z);
	double cosr_cosp = +1.0 - 2.0 * (q->x * q->x + q->y * q->y);
	*roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q->w * q->y - q->z * q->x);
  if (fabs(sinp) >= 1.0) {
    *pitch = copysign(M_PI / 2.0, sinp);  // use 90 degrees if out of range
    ROS_INFO("WARN: sin(pitch) > 1.0");
  } else {
    *pitch = asin(sinp);
  }

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q->w * q->z + q->x * q->y);
	double cosy_cosp = +1.0 - 2.0 * (q->y * q->y + q->z * q->z);
	*yaw = atan2(siny_cosp, cosy_cosp);
}



double offb_integrate( double prevTotal, double prevVal, double curVal, double t0, double t1) {
  return prevTotal + (prevVal + curVal) / 2.0 * (t1 - t0);
}
