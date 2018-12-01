#ifndef OFFB_CONFIG_INCLUDED
#define OFFB_CONFIG_INCLUDED

// #define OFFB_CONTROLLER_MODE_SIMPLE
#define OFFB_CONTROLLER_MODE_NAV
#define OFFB_WAIT_FOR_ARM      // Comment out for in-node arming

#ifdef DT_BUILD_DEV
  // #define OFFB_SHOW_VISUALS
  // #define OFFB_VERBOSE
#endif


// #define OFFB_FLIGHT_LOOP_RATE   DT_LOOP_RATE    // Hz
#define OFFB_START_LOOP_RATE    3    // Hz
#define OFFB_ARM_TIMEOUT       10    // sec
#define OFFB_GEN_TIMEOUT        5    // sec
#define OFFB_ZEROALT_TIME       1.0  // sec
#define OFFB_DEBUG_START_DELAY  1.0  // sec

#define OFFB_MIN_FLIGHT_ALT    0.1   // m, less is considered on ground

#define OFFB_CAM_ANGLE      0.4368   // Camera pitch from ground normal
#define OFFB_CAM_FOV_X     0.69813   // 1.39626
#define OFFB_CAM_FOV_Y     0.52360   // 1.04720
#define OFFB_CAM_SIZE_X      640.0
#define OFFB_CAM_SIZE_Y      480.0

#define DEG                    0.017453293  // deg2rad conversion

// Altitude PID tuning, input rel alt, output throttle
#define OFFB_PID_ALT_F                0.0
#define OFFB_PID_ALT_DTC              1.0
#define OFFB_PID_ALT_DK               1.0

// Pitch PID tuning, intput Y metres, output rad
#define OFFB_PID_PITCH_BIAS              0.0  // output bias
#define OFFB_PID_PITCH_I                 0.0
#define OFFB_PID_PITCH_F                 0.0
#define OFFB_PID_PITCH_DTC               1.0
#define OFFB_PID_PITCH_DK                1.0

// Roll PID tuning, input X px, output rad
#define OFFB_PID_ROLL_BIAS               0.0        // output bias
#define OFFB_PID_ROLL_I                  0.00
#define OFFB_PID_ROLL_F                  0.0
#define OFFB_PID_ROLL_DTC                1.0
#define OFFB_PID_ROLL_DK                 1.0

// Yaw Rate PID tuning, input rad, output rad/s
#define OFFB_PID_YAW_BIAS                0.0        // output bias
#define OFFB_PID_YAW_I                   0.0
#define OFFB_PID_YAW_F                   0.0
#define OFFB_PID_YAW_DTC                 1.0
#define OFFB_PID_YAW_DK                  1.0


// Utility macros
#define SET_BIT(a, b)     ((a) |= (b))
#define CLEAR_BIT(a, b)   ((a) &= (~(b)))
#define GET_BIT(a, b)     (((a) & (b)) > 0)
#define SGNI(x)           ((0 < (x)) - ((x) < 0))
#define SGNF(x)           ((0.0f < (x)) - ((x) < 0.0f))
#endif
