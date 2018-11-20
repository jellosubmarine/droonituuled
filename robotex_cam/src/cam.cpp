#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <numeric>
#include <algorithm>

#include "dt_config.hpp"
#include "cam.hpp"

using namespace std;
using namespace cv;

#ifdef DT_BUILD_DEV
#pragma message "robotex_cam dev build"
#else
#pragma message "robotex_cam live build"
#endif


// Camera image retreival callback
void Visuals::imgcb(const sensor_msgs::ImageConstPtr &msg) {
  frame_time = ros::Time::now();
  frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
  image_flag = true;
}


// Constructor
Visuals::Visuals(ros::NodeHandle &nh) : it(nh) {
  image_flag = false;

  // Read parameters (third argument is default value, if unable to fetch parameter)
  readParam(nh, "threshold/saturation", &rp_sat_thr, 50);
  readParam(nh, "of/refresh_interval", &rp_of_refresh_int, 15);
  
  image_sub = it.subscribe("/camera/color/image_raw", 1, &Visuals::imgcb, this);
  point_pub = nh.advertise<mavros_msgs::PositionTarget>(DT_CAM_TOPIC, 1);
}

template<typename T>
void Visuals::readParam(const ros::NodeHandle& nh, const std::string& param_name, T* var, const T& defaultVal) {
  if (nh.param(param_name, *var, defaultVal)) {
    ROS_INFO_STREAM("Got param: " << param_name << " = " << *var);
  } else {
    ROS_ERROR_STREAM("Failed to get param " << param_name << ", default = " << defaultVal);
  }
}


#ifdef DT_BUILD_DEV
  Visuals::~Visuals() { destroyWindow(CAM_VIS_NAME); }
#endif



/* SETUP AND START */
void Visuals::run() {
  ros::Rate rate(CAM_LOOP_RATE);

  // Wait for first frame to arrive
  while (ros::ok() && !image_flag) {
    ros::spinOnce();
    rate.sleep();
  }

  // Init optical flow
  cvtColor(frame, frame_gray_masked_old, CV_BGR2GRAY);
  frame_old_time = frame_time;
  image_flag = false;

  // Ensure clean storage
  frame_gray = cv::Mat::zeros(frame.rows, frame.cols, CV_8U);
  frame_gray_masked = cv::Mat::zeros(frame.rows, frame.cols, CV_8U);
  frame_gray_masked_old = cv::Mat::zeros(frame.rows, frame.cols, CV_8U);

  // feature_detector = cv::FastFeatureDetector::create(CAM_FAST_THRESHOLD, true);
  of_status.reserve(CAM_GF_MAX_POINTS);
  of_criteria = cvTermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
  of_win_size = cvSize(15, 15);
  of_refresh_counter = rp_of_refresh_int;

  // Create image mask
  edge_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
  edge_mask(Rect(CAM_FRAME_MASK_WIDTH, 0,
                 edge_mask.cols - 2 * CAM_FRAME_MASK_WIDTH,
                 edge_mask.rows)) = Scalar(255);

  mask3 = cv::Mat(frame.size(), CV_8UC3);

  // Main processing loop
  while (ros::ok()) {
    if (image_flag) {
      process();
      image_flag = false;
    }

    ros::spinOnce();
    rate.sleep();
  }
}

// MAIN
int main(int argc, char **argv) {
  ros::init(argc, argv, "robotex_cam");
  ros::NodeHandle nh;
  Visuals node(nh);

#ifdef DT_BUILD_DEV
  namedWindow(CAM_VIS_NAME, WINDOW_AUTOSIZE);
  startWindowThread();
#endif

  node.run();
  return 0;
}
