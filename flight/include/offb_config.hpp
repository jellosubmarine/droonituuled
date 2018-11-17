#ifndef OFFB_CONFIG_INCLUDED
#define OFFB_CONFIG_INCLUDED

//#define OFFB_CONTROLLER_MODE_SIMPLE
#define OFFB_CONTROLLER_MODE_NAV
#define OFFB_WAIT_FOR_ARM      // Comment out for in-node arming

#ifdef DT_BUILD_DEV
  //#define OFFB_SHOW_VISUALS
#endif


#define OFFB_FLIGHT_LOOP_RATE  10   // Hz
#define OFFB_START_LOOP_RATE    3   // Hz
#define OFFB_ARM_TIMEOUT       10   // sec
#define OFFB_GEN_TIMEOUT        5   // sec
#define OFFB_ZEROALT_TIME       1.0 // sec
#define OFFB_DEBUG_START_DELAY  1.0 // sec

#define OFFB_MIN_FLIGHT_ALT    0.1 // m, less is considered on ground
#define OFFB_NAV_ALT_MARGIN    0.3 // m, navigate when alt +- margin
#define OFFB_NAV_MAX_CLIMB     0.2 // m/s
#define OFFB_ABORT_ALT         2.0 // m, max allowed altitude
#define OFFB_ABORT_LOST_TIME  10.0 // s, max time to be lost

#define OFFB_CAM_ANGLE      0.4368 // Camera pitch from ground normal
#define OFFB_CAM_FOV_X     0.69813 //1.39626
#define OFFB_CAM_FOV_Y     0.52360 //1.04720
#define OFFB_CAM_SIZE_X      640.0
#define OFFB_CAM_SIZE_Y      480.0
#define OFFB_CAM_OFFSET_X    320.0
#define OFFB_CAM_OFFSET_Y      0.0

#define DEG 0.017453293 // deg2rad conversion
#define OFFB_ROLL_YAW_COUPL           0.001
#define OFFB_LOST_YAW_RATE          (30*DEG)

// Altitude PID tuning, input rel alt, output throttle
#define OFFB_ALT_TARGET               1.5 // metres
#define OFFB_PID_ALT_MAX_OUTPUT       1.0 // throttle
#define OFFB_PID_ALT_MIN_OUTPUT       0.0 // throttle
#define OFFB_PID_ALT_MAX_OUTPUT_RAMP  1.0 // throttle per sec
#define OFFB_PID_ALT_BIAS             0.5 //0.0 // output bias
#define OFFB_PID_ALT_P                0.07 //0.20 //0.2
#define OFFB_PID_ALT_I                0.00 //0.06 //0.06
#define OFFB_PID_ALT_D                0.07 //0.07 //0.02
#define OFFB_PID_ALT_F                0.0
#define OFFB_PID_ALT_DTC              1.0
#define OFFB_PID_ALT_DK               1.0

// Pitch PID tuning, intput Y metres, output rad
#define OFFB_PITCH_TARGET                0.5 // metres ahead
#define OFFB_PID_PITCH_MAX_OUTPUT      (10*DEG) // pitch angle rad
#define OFFB_PID_PITCH_MIN_OUTPUT     (-10*DEG)
#define OFFB_PID_PITCH_MAX_OUTPUT_RAMP (10*DEG)
#define OFFB_PID_PITCH_BIAS              0.0 // output bias
#define OFFB_PID_PITCH_P               (-0.3/9.81)
#define OFFB_PID_PITCH_I                 0.0
#define OFFB_PID_PITCH_D               (+1.0/9.81)
#define OFFB_PID_PITCH_F                 0.0
#define OFFB_PID_PITCH_DTC               1.0
#define OFFB_PID_PITCH_DK                1.0

// Roll PID tuning, input X px, output rad
#define OFFB_ROLL_TARGET                 0.0 // metres right
#define OFFB_PID_ROLL_MAX_OUTPUT       (10*DEG) // Bank angle rad
#define OFFB_PID_ROLL_MIN_OUTPUT      (-10*DEG)
#define OFFB_PID_ROLL_MAX_OUTPUT_RAMP  (10*DEG)
#define OFFB_PID_ROLL_BIAS               0.0 // output bias
#define OFFB_PID_ROLL_P                (-0.3/9.81) // bank from accel
#define OFFB_PID_ROLL_I                  0.00
#define OFFB_PID_ROLL_D                (+1.0/9.81)
#define OFFB_PID_ROLL_F                  0.0
#define OFFB_PID_ROLL_DTC                1.0
#define OFFB_PID_ROLL_DK                 1.0

// Yaw Rate PID tuning, input rad, output rad/s
#define OFFB_YAW_TARGET                  0.0 // brg to point, rad
#define OFFB_PID_YAW_MAX_OUTPUT        (30*DEG)
#define OFFB_PID_YAW_MIN_OUTPUT       (-30*DEG)
#define OFFB_PID_YAW_MAX_OUTPUT_RAMP   (15*DEG)
#define OFFB_PID_YAW_BIAS                0.0 // output bias
#define OFFB_PID_YAW_P                  -0.1
#define OFFB_PID_YAW_I                   0.0
#define OFFB_PID_YAW_D                  -0.3
#define OFFB_PID_YAW_F                   0.0
#define OFFB_PID_YAW_DTC                 1.0
#define OFFB_PID_YAW_DK                  1.0


// Utility macros
#define SET_BIT(a,b)     ((a) |= (b))
#define CLEAR_BIT(a,b)   ((a) &= (~(b)))
#define GET_BIT(a,b)     (((a) & (b)) > 0)

#endif
