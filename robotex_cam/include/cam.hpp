#ifndef DT_CAM_INCLUDED
#define DT_CAM_INCLUDED

#include <string>
#include <vector>
#include "image_transport/image_transport.h"
#include "mavros_msgs/PositionTarget.h"
#include "opencv2/opencv.hpp"

#include "dt_config.hpp"

#ifdef DT_BUILD_DEV
  // #define CAM_VERBOSE

  #ifdef CAM_VERBOSE
    // #define CAM_SHOW_FPS
  #endif
#endif

#define CAM_LOOP_RATE           15  // Hz

#define CAM_FRAME_WIDTH        640
#define CAM_FRAME_HEIGHT       480
#define CAM_FRAME_OFFSET_X     320
#define CAM_FRAME_OFFSET_Y     240

#define CAM_CANNY_THR_LOW      121
#define CAM_CANNY_THR_HIGH     (CAM_CANNY_THR_LOW * 1.5)
#define CAM_CANNY_KERNEL_SIZE    3
#define CAM_BLUR_SIZE            3
#define CAM_CONTOUR_LIM_LOW   1000
#define CAM_CONTOUR_LIM_HIGH 10000
#define CAM_CONTOUR_MIN_DIST2   25.0

#define CAM_OF_MAX_LEVELS        2  // OF pyramid levels
#define CAM_FAST_THRESHOLD      20
#define CAM_GF_MAX_POINTS       20
#define CAM_GF_QUALITY           0.3
#define CAM_GF_MIN_DIST         10
#define CAM_GF_BLOCK_SIZE        7

#define CAM_VIS_NAME           "robotex_cam visualisation"
#define CAM_VIS_LENGTH         100  // Visualisation arbitrary line length


class Visuals {
public:
  Visuals(ros::NodeHandle &nh);
  #ifdef DT_BUILD_DEV
    ~Visuals();
  #endif

  void imgcb(const sensor_msgs::ImageConstPtr &msg);
  void run();

protected:
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  ros::Publisher point_pub;
  mavros_msgs::PositionTarget msg;

private:
  void process();
  void removeOutliers();
  void weighCentroids();
  void dynamicMask();
  float hdgFromPca();
  float hdgFromBottomPoint();
  float hdgFromLineFit();

  // Ros parameters
  template<typename T> void readParam(const ros::NodeHandle& nh, const std::string& param_name, T* var, const T& defaultVal);
  int rp_mask_sat_thr;
  int rp_of_refresh_int;
  int rp_dynmask_max_width, rp_dynmask_req_contours;
  float rp_contour_outlier;

  // Frame storage
  bool image_flag;
  cv::Mat frame, frame_HSV, frame_gray, frame_gray_masked, frame_gray_masked_old;
  cv::Mat hsv_mask, edge_mask, total_mask, outframe;
  ros::Time frame_time, frame_old_time;
  int dynmask_width;
  
  // Optical flow
  std::vector<cv::Point2f> of_old_points, of_new_points;
  std::vector<uchar> of_status;
  cv::Mat of_err;
  int of_refresh_counter;
  // cv::Ptr<cv::FastFeatureDetector> feature_detector;
  CvTermCriteria of_criteria;
  CvSize of_win_size;
  // std::vector<cv::KeyPoint> of_keypoints;
  std::vector<cv::Point2f> of_velocities;
  cv::Point2f of_mean_velocity;

  // Image processing storage
  cv::Mat blurr, erod, dil, canny_output;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Moments> mu;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Point2f> centroids, weighted_centroids;
  std::vector<float> line_data;
  cv::Mat distances;
  cv::Point2f bottom_centroid, mean_centroid;
  std::vector<float> c_x, c_y;
  float Z, Z1, Z2, min_dist_abs, min_dist_cur;
  int i, j, s;


  #ifdef DT_BUILD_DEV
    cv::Mat mask3;
  #endif
};

#endif
