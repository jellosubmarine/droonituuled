#ifndef DT_CAM_INCLUDED
#define DT_CAM_INCLUDED

#include "image_transport/image_transport.h"
//#include "geometry_msgs/QuaternionStamped.h"
#include "mavros_msgs/PositionTarget.h"
#include "opencv2/opencv.hpp"

#define CAM_LOOP_RATE           15  // Hz

#define CAM_FRAME_WIDTH        640
#define CAM_FRAME_HEIGHT       480
#define CAM_FRAME_OFFSET_X     320
#define CAM_FRAME_OFFSET_Y     240
#define CAM_FRAME_MASK_WIDTH   50

#define CAM_CANNY_THR_LOW      121
#define CAM_CANNY_THR_HIGH     (CAM_CANNY_THR_LOW * 1.5)
#define CAM_CANNY_KERNEL_SIZE    3
#define CAM_BLUR_SIZE            3
#define CAM_CONTOUR_LIM_LOW   1000
#define CAM_CONTOUR_LIM_HIGH 10000
#define CAM_CONTOUR_OUTLIER      1.5  // Distance factor
#define CAM_SATURATION_THRESH   50

#define CAM_OF_REFRESH_INTERVAL 15  // frames
#define CAM_OF_MAX_LEVELS        2  // OF pyramid levels
#define CAM_FAST_THRESHOLD      20
#define CAM_GF_MAX_POINTS       20
#define CAM_GF_QUALITY           0.3
#define CAM_GF_MIN_DIST         10
#define CAM_GF_BLOCK_SIZE        7

#define CAM_WINDOW_NAME  "robotex_cam visualisation"


class Visuals {
public:
  Visuals(ros::NodeHandle &nh_);
  ~Visuals();

  void imgcb(const sensor_msgs::ImageConstPtr &msg);
  void process();
  void run();

protected:
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub;
  ros::Publisher point_pub;
  mavros_msgs::PositionTarget msg;
  //geometry_msgs::QuaternionStamped msg;

private:
  bool image_flag;

  // Frame storage
  cv::Mat frame, frame_gray, frame_gray_old, frame_HSV;
  ros::Time frame_time, frame_old_time;

  // Optical flow
  std::vector<cv::Point2f> of_old_points;
  std::vector<cv::Point2f> of_new_points;
  std::vector<uchar> of_status; // TODO: maybe init to CAM_OF_MAX_POINTS
  cv::Mat of_err;
  int of_refresh_counter;
  cv::Ptr<cv::FastFeatureDetector> feature_detector;
  CvTermCriteria of_criteria;
  CvSize of_win_size;
  std::vector<cv::KeyPoint> of_keypoints;
  std::vector<cv::Point2f> of_velocities;
  cv::Point2f of_mean_velocity;

  // Image processing storage
  cv::Mat img_gray, img_bw, img_final;
  cv::Mat blurr, erod, dil, proc;;
  cv::Mat mask,gray_mask, canny_output, masked_gray;

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Moments> mu;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Point2f> centroids;
  cv::Mat distances;
  cv::Point2f bottom_centroid, mean_centroid;
  std::vector<float> c_x, c_y;
  float Z;
  int W;
  #ifdef DT_BUILD_DEV
    cv::Mat mask3;
  #endif
};

#endif
