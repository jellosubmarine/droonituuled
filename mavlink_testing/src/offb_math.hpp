#ifndef OFFB_MATH_INCLUDED
#define OFFB_MATH_INCLUDED

#include "geometry_msgs/Quaternion.h"

geometry_msgs::Quaternion offb_euler2quat(double pitch, double roll, double yaw);
double offb_integrate( double prevTotal, double prevVal, double curVal, double t0, double t1);

#endif
