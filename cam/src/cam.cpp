//#include <ctime>
#include <iostream>
#include <numeric>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "dt_config.hpp"
#include "cam.hpp"

using namespace std;
using namespace cv;


#ifdef DT_BUILD_DEV
  #pragma message "robotex_cam dev build"
#else
  #pragma message "robotex_cam live build"
#endif


/* CONSTRUCTORS AND CALLBACKS */

Visuals::Visuals(ros::NodeHandle &nh_) : it_(nh_) {
  image_flag = false;
  image_sub = it_.subscribe("/camera/color/image_raw", 1, &Visuals::imgcb, this);
  point_pub = nh_.advertise<mavros_msgs::PositionTarget>(DT_CAM_TOPIC, 1);
}

#ifdef DT_BUILD_DEV
  Visuals::~Visuals() { destroyWindow("Contours"); }
#endif

void Visuals::imgcb(const sensor_msgs::ImageConstPtr &msg) {
  frame_time = ros::Time::now();
  frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
  cvtColor(frame, frame_gray, CV_BGR2GRAY); // no risk of overwriting prematurely
  image_flag = true;
}


/* IMAGE PROCESSING */

void Visuals::process() {
  #ifdef DT_BUILD_DEV
    // Start time for frame rate calculation
    float startTime = ros::Time::now().toSec(); //std::clock_t start( std::clock() );
  #endif


  // Update optical flow tracked points at given interval
  if (of_refresh_counter >= CAM_OF_REFRESH_INTERVAL) {
    of_refresh_counter = 0;
    of_old_points.clear();

    // Detect keypoints and extract raw point data
    /*
    feature_detector->detect(frame_gray_old, of_keypoints);
    for (size_t i = 0; i < of_keypoints.size(); ++i){
      of_old_points.push_back(of_keypoints[i].pt);
    }
    */
    goodFeaturesToTrack(frame_gray_old, of_old_points, CAM_GF_MAX_POINTS,
        CAM_GF_QUALITY, CAM_GF_MIN_DIST, cv::noArray(), CAM_GF_BLOCK_SIZE);

    ROS_INFO_STREAM("Found " << of_old_points.size() << " OF points" );

  }


  // Calculate optical flow
  if (!of_old_points.empty()) {
    calcOpticalFlowPyrLK( frame_gray_old, frame_gray,
                          of_old_points, of_new_points,
                          of_status, of_err, of_win_size,
                          CAM_OF_MAX_LEVELS, of_criteria );
  }

  // select good points
  //vector<Point2f> goodNew(st.size());
  //vector<Point2f> goodOld(st.size());

  // Calculate velocities
  of_velocities.clear();
  of_mean_velocity.x = 0.0;
  of_mean_velocity.y = 0.0;

  for (size_t i = 0; i < of_status.size(); ++i) {
    if (of_status[i]) {
      of_velocities.push_back( cv::Point2f(
        (of_new_points[i].x - of_old_points[i].x),
        (of_old_points[i].y - of_new_points[i].y)  // Correct frame orientation
      ) );

      of_mean_velocity.x += of_velocities[i].x;
      of_mean_velocity.y += of_velocities[i].y;
    }
  }

  if (of_velocities.size() > 0) {
    of_mean_velocity.x = of_mean_velocity.x / float(of_velocities.size()) /
        (frame_time - frame_old_time).toSec() / float(CAM_FRAME_WIDTH) * 2.0f;  // Normalised to half frame
    of_mean_velocity.y = of_mean_velocity.y / float(of_velocities.size()) /
        (frame_time - frame_old_time).toSec() / float(CAM_FRAME_HEIGHT) * 2.0f;  // Normalised to half frame
  }

  // Finished with OF


  // Process and find contours
  blur(frame_gray, blurr, Size(CAM_BLUR_SIZE, CAM_BLUR_SIZE), Point(-1, -1));       // Blur image
  Canny(blurr, dil, CAM_CANNY_THR_LOW, CAM_CANNY_THR_HIGH, CAM_CANNY_KERNEL_SIZE ); // Find edges
  dilate(dil, erod, Mat(), Point(-1, -1), 4, 1, 1);                                 // Get rid of holes
  //erode(dil, dil, Mat(), Point(-1, -1), 4, 1, 1);

  erod.copyTo(canny_output, mask);
  findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0)); // Get contours


  // Clear storage vectors
  mu.clear();
  avg_x.clear();
  avg_y.clear();
  #ifdef DT_BUILD_DEV
    mc.clear();
  #endif

  // Calculate contour moments
  for (size_t i = 0; i < contours.size(); ++i) { // Look through all contours
    if (contourArea(contours[i]) > CAM_CONTOUR_LIM_LOW &&
        contourArea(contours[i]) < CAM_CONTOUR_LIM_HIGH)
    {
      mu.push_back(moments(contours[i], false));
    }
  }

  // Calculate centroids from moments
  for (size_t i = 0; i < mu.size(); i++) {
    avg_x.push_back( float(mu[i].m10 / mu[i].m00) );
    avg_y.push_back( float(mu[i].m01 / mu[i].m00) );

    #ifdef DT_BUILD_DEV
      // For visualisation
      mc.push_back( Point2f( static_cast<float>(mu[i].m10 / mu[i].m00),
                             static_cast<float>(mu[i].m01 / mu[i].m00) ) );
    #endif

    if (i==0){
        bot_X = static_cast<float>(mu[i].m10 / mu[i].m00);
        bot_Y = static_cast<float>(mu[i].m01 / mu[i].m00);
    }
  }

  // Calculate mean centroid
  sum_x = accumulate(avg_x.begin(), avg_x.end(), 0); // Sum all centroids' x values
  sum_y = accumulate(avg_y.begin(), avg_y.end(), 0); // Sum all centroids' y values

  if (avg_x.size() > 0) {
    X = (sum_x / float(avg_x.size())); // Average x coordinate
    Y = (sum_y / float(avg_y.size())); // Average y coordinate
  }
  else {
    // No contours found
    W = 0;
    X = -1.0;
    Y = -1.0;
    Z = -1.0;

    #ifdef DT_BUILD_DEV
      //ROS_INFO("No contours detected.");
    #endif
  }

  // Publish average point x, y,
  // directon from bottom point to average point,
  // number of found points,
  // of mean velocity

  if( X >= 0 and Y >= 0) {
    X -= CAM_FRAME_OFFSET_X;
    Y = CAM_FRAME_HEIGHT - Y - CAM_FRAME_OFFSET_Y; // Conv Y to bottom up
    bot_X -= CAM_FRAME_OFFSET_X;
    bot_Y = CAM_FRAME_HEIGHT - bot_Y - CAM_FRAME_OFFSET_Y;
    Z = atan2(X-bot_X,Y-bot_Y);
    W = avg_x.size();
  }

  //msg.header.frame_id = "frames";
  msg.header.stamp = frame_time;

  // mean centroid
  msg.position.x = X / float(CAM_FRAME_WIDTH) * 2.0;  // Norm to half frame
  msg.position.y = Y / float(CAM_FRAME_HEIGHT) * 2.0;
  msg.position.z = W; // Number of visible points

  // direction of centroids
  msg.yaw = Z;

  // OF velocity
  msg.velocity.x = of_mean_velocity.x;
  msg.velocity.y = of_mean_velocity.y;

  point_pub.publish(msg);


  // DRAW VISUALS
  #ifdef DT_BUILD_DEV
    //canny_output = canny_output * 0.2;

    // Draw centroid markers
    for (size_t i = 1; i < mc.size(); i++) {
        circle(frame, mc[i], 4, Scalar(100,80,50), 2, 8, 0);
    }

    // Draw average and bottom markers
    if (mc.size() > 0) {
      Point2f XY(
        static_cast<float>(X+CAM_FRAME_OFFSET_X),
        static_cast<float>(CAM_FRAME_HEIGHT - Y - CAM_FRAME_OFFSET_Y));
      circle(frame, XY, 10, Scalar(30,200,50), -1, 8, 0);
      circle(frame, mc[0], 7, Scalar(30,150,30), -1, 8, 0);
    }

    // Draw mean velocity vector
    cv::arrowedLine( frame,
      cv::Point(
        CAM_FRAME_OFFSET_X,
        CAM_FRAME_HEIGHT - CAM_FRAME_OFFSET_Y
      ),
      cv::Point(
        CAM_FRAME_OFFSET_X + of_mean_velocity.x * CAM_FRAME_WIDTH,
        CAM_FRAME_HEIGHT - (CAM_FRAME_OFFSET_Y + of_mean_velocity.y * CAM_FRAME_HEIGHT)
      ),
      Scalar(40,40,200), 2, 8, 0, 0.1
    );

    // Draw detected features
    for (int i = 0; i < of_old_points.size(); ++i) {
      cv::drawMarker(
        frame, of_old_points[i], Scalar(200,30,30),
        cv::MARKER_TILTED_CROSS, 5, 1, 8
      );
    }

    for (int i = 0; i < of_new_points.size(); ++i) {
      cv::drawMarker(
        frame, of_new_points[i], Scalar(50,200,30),
        cv::MARKER_TILTED_CROSS, 5, 1, 8
      );
    }

    imshow(CAM_WINDOW_NAME, frame);
    waitKey(1);

    /*
      if (W==1){
        ROS_INFO("Msg Sent.");
        ROS_INFO("Subscribers: %d", point_pub.getNumSubscribers());
      }
    */

    // Calculate and display fps
    float fps = 1.0f / (ros::Time::now().toSec() - startTime); // (float) CLOCKS_PER_SEC; // Get FPS here
    cout << "ros fps: " << fps << '\n';
  #endif


  // Copy data for next iteration
  frame_gray.copyTo(frame_gray_old);
  frame_old_time = frame_time;
  of_old_points = of_new_points;
  ++of_refresh_counter;
}


/* SETUP AND START */

void Visuals::run() {
  ros::Rate rate(CAM_LOOP_RATE);

  // Wait for first frame to arrive
  while(ros::ok() && !image_flag) {
    ros::spinOnce();
    rate.sleep();
  }

  // Init optical flow
  cvtColor(frame, frame_gray_old, CV_BGR2GRAY);
  frame_old_time = frame_time;
  image_flag = false;

  feature_detector = cv::FastFeatureDetector::create(CAM_FAST_THRESHOLD, true);
  of_criteria = cvTermCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
  of_win_size = cvSize(15,15);
  of_refresh_counter = CAM_OF_REFRESH_INTERVAL;

  // Create image mask
  mask = cv::Mat::zeros(frame.size(), CV_8UC1);
  mask( Rect( CAM_FRAME_MASK_WIDTH, 0,
              mask.cols - 2 * CAM_FRAME_MASK_WIDTH,
              mask.rows )
  ) = Scalar(1);


  // Main processing loop
  while (ros::ok()) {
    if (image_flag) {
      process();
      image_flag = false;
    }

    ros::spinOnce();
    rate.sleep();
  }
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotex_cam");
    ros::NodeHandle nh;
    Visuals node(nh);

    #ifdef DT_BUILD_DEV
      namedWindow(CAM_WINDOW_NAME, WINDOW_AUTOSIZE);
      startWindowThread();
    #endif

    node.run();
    return 0;
}
