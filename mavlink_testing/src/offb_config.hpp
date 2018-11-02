#ifndef OFFB_CONFIG_INCLUDED
#define OFFB_CONFIG_INCLUDED


#define OFFB_FLIGHT_LOOP_RATE 10  // Hz
#define OFFB_START_LOOP_RATE  3   // Hz
#define OFFB_TIMEOUT          5   // sec
#define OFFB_ZEROALT_TIME     1.0 // sec
#define OFFB_DEBUG_START_DELAY 5.0 // sec

// Altitude PID tuning
#define OFFB_ALT_TARGET               1.5 // metres
#define OFFB_PID_ALT_MAX_OUTPUT       1.0 // throttle
#define OFFB_PID_ALT_MIN_OUTPUT       0.0 // throttle
#define OFFB_PID_ALT_MAX_OUTPUT_RAMP  1.0 // throttle per sec
#define OFFB_PID_ALT_P                0.2 //0.2
#define OFFB_PID_ALT_I                0.06 //0.06
#define OFFB_PID_ALT_D                0.0 //0.02
#define OFFB_PID_ALT_F                0.0
#define OFFB_PID_ALT_DTC              1.0
#define OFFB_PID_ALT_DK               1.0


#endif
