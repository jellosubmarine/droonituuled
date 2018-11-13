#ifndef OFFB_MATH_INCLUDED
#define OFFB_MATH_INCLUDED

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"

void offb_cam2floor( double x, double y, double alt,
                     double cam_pitch, double cam_fov_x, double cam_fov_y,
                     geometry_msgs::Point *ret );

void offb_euler2quat(double roll, double pitch, double yaw, geometry_msgs::Quaternion *ret);

void offb_quat2euler( const geometry_msgs::Quaternion *q,
                      double *roll, double *pitch, double *yaw );


double offb_integrate( double prevTotal, double prevVal, double curVal, double t0, double t1);

#endif
