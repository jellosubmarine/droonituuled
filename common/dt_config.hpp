#ifndef DT_CONFIG_INCLUDED
#define DT_CONFIG_INCLUDED

// Common config for Droonituuled

// Used to compile software for quadcopter
// Comment out for development (simulation) build
//#define DT_BUILD_LIVE

#define DT_CAM_TOPIC       "/dt/cam_point_vector"
#define DT_DEBUG_TOPIC     "/dt/debug"



// Don't change these

#ifndef DT_BUILD_LIVE
  #define DT_BUILD_DEV
#endif

#endif
