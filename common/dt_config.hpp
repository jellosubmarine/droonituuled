#ifndef DT_CONFIG_INCLUDED
#define DT_CONFIG_INCLUDED

// Common config for Droonituuled

// Used to compile software for quadcopter
// Comment out for development (simulation) build
//#define DT_BUILD_LIVE

#define DT_CAM_TOPIC       "/dt/cam_topic" // mavros_msgs::PositionTarget
#define DT_DEBUG_TOPIC     "/dt/debug"     // geometry_msgs::Quaternion



// Don't change these

#ifndef DT_BUILD_LIVE
  #define DT_BUILD_DEV
#endif

#endif
