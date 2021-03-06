#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <numeric>
#include <algorithm>

#include "dt_config.hpp"
#include "cam.hpp"


#ifdef DT_BUILD_DEV
  #pragma message "robotex_cam dev build"
// #else
//   #pragma message "robotex_cam live build"
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
  readParam(nh, "loop_rate", &rp_loop_rate, 10);
  readParam(nh, "of/refresh_interval", &rp_of_refresh_int, 15);
  readParam(nh, "frame/mask/saturation_thr", &rp_mask_sat_thr, 50);
  readParam(nh, "contour/rect/min_size", &rp_rect_min_size, 1000);
  readParam(nh, "contour/rect/max_size", &rp_rect_max_size, 4000);
  readParam(nh, "contour/rect/min_ratio", &rp_rect_min_ratio, 2.0f);
  readParam(nh, "contour/rect/max_ratio", &rp_rect_max_ratio, 5.0f);
  readParam(nh, "contour/rect/max_angle", &rp_rect_max_angle, 55.0f);

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
  Visuals::~Visuals() { cv::destroyWindow(CAM_VIS_NAME); }
#endif



/* SETUP AND START */
void Visuals::run() {
  ros::Rate rate(rp_loop_rate);

  // Wait for first frame to arrive
  while (ros::ok() && !image_flag) {
    ros::spinOnce();
    rate.sleep();
  }

  // Init optical flow
  cv::cvtColor(frame, frame_gray_masked_old, CV_BGR2GRAY);
  frame_old_time = frame_time;
  image_flag = false;

  of_status.reserve(CAM_GF_MAX_POINTS);
  of_old_points.reserve(CAM_GF_MAX_POINTS);
  of_new_points.reserve(CAM_GF_MAX_POINTS);
  of_velocities.reserve(CAM_GF_MAX_POINTS);
  of_criteria = cvTermCriteria(
    cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
  of_win_size = cvSize(15, 15);
  of_refresh_counter = rp_of_refresh_int;

  // Ensure clean storage
  frame_gray = cv::Mat::zeros(frame.rows, frame.cols, CV_8U);
  frame_gray_masked = cv::Mat::zeros(frame.rows, frame.cols, CV_8U);
  frame_gray_masked_old = cv::Mat::zeros(frame.rows, frame.cols, CV_8U);
  contours.reserve(20);
  contour_rects.reserve(20);
  centroids.reserve(10);

  // Create image mask
  edge_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
  cv::ellipse(edge_mask, cv::RotatedRect(
    cv::Point2f((CAM_FRAME_WIDTH-1)/2.0f, (CAM_FRAME_HEIGHT-1)/2.0f),
    cv::Size2f(CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT), 0.0f),
    cv::Scalar(255), -1, 8);

  #ifdef DT_BUILD_DEV
    mask3 = cv::Mat(frame.size(), CV_8UC3);
  #endif

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
  ros::init(argc, argv, "robotex_cam_spectator");
  ros::NodeHandle nh;
  Visuals node(nh);

#ifdef DT_BUILD_DEV
  cv::namedWindow(CAM_VIS_NAME, cv::WINDOW_AUTOSIZE);
  cv::startWindowThread();
#endif

  node.run();
  return 0;
}
