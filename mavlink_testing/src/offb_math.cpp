#include "offb_math.hpp"
#include <cmath>


// Converts yaw-pitch-roll to unit quaternion
// Angles in radians
geometry_msgs::Quaternion offb_euler2quat(double pitch, double roll, double yaw) {
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cy * cr * cp + sy * sr * sp;
    q.x = cy * sr * cp - sy * cr * sp;
    q.y = cy * cr * sp + sy * sr * cp;
    q.z = sy * cr * cp - cy * sr * sp;
    return q;
}


double offb_integrate( double prevTotal, double prevVal, double curVal, double t0, double t1) {
  return prevTotal + (prevVal + curVal) / 2.0 * (t1 - t0);
}
