#include <numeric>

#include "dt_config.hpp"
#include "cam.hpp"
#include "cam_algo.hpp"

// using namespace std;
// using namespace cv;



// Find and remove outliers using brute force
void Visuals::removeOutliers() {
  s = centroids.size();

  distances = cv::Mat::zeros(s, s, CV_32F);
  min_dist_abs = FLT_MAX;  // Min distance betw two centroids

  // Determine minimum nearest neighbour distance
  for (i = 0; i < s; ++i) {
    for (j = i+1; j < s; ++j) {
      distances.at<float>(i, j) = dist2(centroids[i], centroids[j]);
      if (distances.at<float>(i, j) < min_dist_abs) {
        min_dist_abs = distances.at<float>(i, j);
      }
    }
  }

  // Find outliers
  for (i = 0; i < s; ++i) {
    min_dist_cur = FLT_MAX;

    // Compare to centroids before this one
    for (j = 0; j < i; ++j) {
      if (distances.at<float>(j, i) < min_dist_cur) {
        min_dist_cur = distances.at<float>(j, i);
      }
    }

    // Compare to centroids after this one
    for (j = i+1; j < s; ++j) {
      if (distances.at<float>(i, j) < min_dist_cur) {
        min_dist_cur = distances.at<float>(i, j);
      }
    }

    // Mark outliers for removal
    if (min_dist_cur > rp_contour_outlier * min_dist_abs) {
      centroids[i].x = FLT_MAX;
    }
  }

  // Remove outliers
  for (i = 0; i < centroids.size(); ++i) {
    if (centroids[i].x == FLT_MAX) {
      ROS_INFO_STREAM(
        "Dropped point [" <<
        convertXCoord(centroids[i].x) << ", " <<
        convertYCoord(centroids[i].y) <<
        "], min dist sq = " << min_dist_abs);

      centroids.erase(centroids.begin() + i);
      contours.erase(contours.begin() + i);
      mu.erase(mu.begin() + i);
      --i;
    }
  }
}


/* IMAGE PROCESSING */
void Visuals::process() {
  #ifdef CAM_SHOW_FPS
    // Start time for frame rate calculation
    float startTime = ros::Time::now().toSec();  // std::clock_t start( std::clock() );
  #endif


  // Color conversion and masking
  cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
  cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
  inRange(frame_HSV, cv::Scalar(0, 0, 0), cv::Scalar(255, rp_mask_sat_thr, 255), hsv_mask);
  bitwise_and(hsv_mask, edge_mask, total_mask);
  bitwise_and(frame_gray, total_mask, frame_gray_masked);


  // Update optical flow tracked points at given interval
  if (of_refresh_counter >= rp_of_refresh_int) {
    of_refresh_counter = 0;
    of_old_points.clear();

    goodFeaturesToTrack(frame_gray_masked_old, of_old_points, CAM_GF_MAX_POINTS,
        CAM_GF_QUALITY, CAM_GF_MIN_DIST, cv::noArray(), CAM_GF_BLOCK_SIZE);

    #ifdef CAM_VERBOSE
      ROS_INFO_STREAM("Found " << of_old_points.size() << " OF points");
    #endif
  }

  // Calculate optical flow
  if (!of_old_points.empty()) {
    calcOpticalFlowPyrLK(frame_gray_masked_old, frame_gray_masked,
                         of_old_points, of_new_points,
                         of_status, of_err, of_win_size,
                         CAM_OF_MAX_LEVELS, of_criteria);
  }

  // Calculate velocities
  of_velocities.clear();
  of_mean_velocity.x = 0.0;
  of_mean_velocity.y = 0.0;

  for (i = 0; i < of_status.size(); ++i) {
    if (of_status[i]) {
      of_velocities.push_back(cv::Point2f(
        (of_new_points[i].x - of_old_points[i].x),
        (of_old_points[i].y - of_new_points[i].y)));  // Correct frame orientation

      of_mean_velocity.x += of_velocities[i].x;
      of_mean_velocity.y += of_velocities[i].y;
    }
  }

  if (of_velocities.size() > 0) {
    of_mean_velocity.x = of_mean_velocity.x / float(of_velocities.size()) /
         (frame_time - frame_old_time).toSec() / float(CAM_FRAME_WIDTH) * 2.0f;   // Normalised to half frame
    of_mean_velocity.y = of_mean_velocity.y / float(of_velocities.size()) /
         (frame_time - frame_old_time).toSec() / float(CAM_FRAME_HEIGHT) * 2.0f;  // Normalised to half frame
  }


  // Process and find contours
  blur(frame_gray_masked, blurr, cv::Size(CAM_BLUR_SIZE, CAM_BLUR_SIZE), cv::Point(-1, -1));  // Blur image
  Canny(blurr, dil, CAM_CANNY_THR_LOW, CAM_CANNY_THR_HIGH, CAM_CANNY_KERNEL_SIZE);    // Find edges
  dilate(dil, canny_output, cv::Mat(), cv::Point(-1, -1), 4, 1, 1);                           // Get rid of holes
  // erode(dil, dil, Mat(), Point(-1, -1), 4, 1, 1);
  findContours(canny_output, contours, cv::RETR_EXTERNAL,
    cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


  // Clear storage vectors
  mu.clear();
  contour_angles.clear();
  contour_rects.clear();
  centroids.clear();
  // weighted_centroids.clear();

  // Filter and process contours
  for (i = 0; i < contours.size(); ++i) {
    // contour_area = cv::contourArea(contours[i]);
    contour_rects.push_back(cv::minAreaRect(contours[i]));

    if (contour_rects[i].size.width > contour_rects[i].size.height) {
      temp = contour_rects[i].size.width;
      contour_rects[i].size.width = contour_rects[i].size.height;
      contour_rects[i].size.height = temp;
      contour_rects[i].angle += 90;  // degrees
    }

    rect_area = float(contour_rects[i].size.width) *
                float(contour_rects[i].size.height);
    rect_ratio = float(contour_rects[i].size.height) /
                 float(contour_rects[i].size.width);
    // if (rect_ratio < 1.0f) rect_ratio = 1.0f / rect_ratio;

    // Check for size limits
    // if (contour_area >= rp_contour_min_size &&
    //     contour_area <= rp_contour_max_size) {

    if (rect_area >= rp_rect_min_size &&
        rect_area <= rp_rect_max_size &&
        rect_ratio >= rp_rect_min_ratio &&
        rect_ratio <= rp_rect_max_ratio &&
        fabs(contour_rects[i].angle) <= rp_rect_max_angle) {

      centroids.push_back(contour_rects[i].center);

      // this->moments = cv::moments(contours[i], false);
      // contour_angles.push_back(contour_angle(this->moments));

      // Check contour angle
      // if (fabs(contour_angles[i]) <= rp_contour_max_angle) {
      //   centroids.push_back(cv::Point2f(
      //     float(this->moments.m10) / float(this->moments.m00),
      //     float(this->moments.m01) / float(this->moments.m00)));
      // } else {
      //   contour_angles.erase(contour_angles.begin() + i);
      //   contours.erase(contours.begin() + i);
      //   contour_rects.erase(contour_rects.begin() + i);
      //   --i;
      // }
    } else {
      contours.erase(contours.begin() + i);
      contour_rects.erase(contour_rects.begin() + i);
      --i;
    }
  }

  // Remove close lying centroids
  /*
  for (i = 0; i < centroids.size(); ++i) {
    for (j = i+1; j < centroids.size(); ++j) {
      if (dist2(centroids[i], centroids[j]) < CAM_CONTOUR_MIN_DIST2) {
        ROS_INFO("Removing duplicate centroid");
        centroids.erase(centroids.begin() + j);
        contours.erase(contours.begin() + j);
        mu.erase(mu.begin() + j);
        --j;
      }
    }
  }
  */

/*
  // todo: maybe use stats for mask width calculation
  c_x.clear();
  c_y.clear();
  for (i = 0; i < centroids.size(); ++i) {
    c_x.push_back(centroids[i].x);
    c_x.push_back(centroids[i].y);
  }
*/
  if (centroids.size()) {
    // removeOutliers();
    // weighCentroids();
    // weighted_centroids = centroids;

    mean_point(centroids, &mean_centroid);
    if (centroids.size() > 1) {
      bottom_centroid.x = convertXCoord(centroids[0].x);
      bottom_centroid.y = convertYCoord(centroids[0].y);
    } else {
      bottom_centroid.x = 0.0;
      bottom_centroid.y = -CAM_FRAME_OFFSET_Y;
    }

    Z = hdgFromBottomPoint();
    // Z = hdgFromLineFit();
    // Z = (Z1+Z2)/2;
    // Z = hdgFromPca();
  } else {
    mean_centroid.x = 0.0;
    mean_centroid.y = 0.0;
    bottom_centroid.x = 0.0;
    bottom_centroid.y = -CAM_FRAME_OFFSET_Y;
    Z = 0.0;
  }

  // Publish average point x, y,
  // directon from bottom point to average point,
  // number of found points,
  // of mean velocity
  msg.header.stamp = frame_time;

  // mean centroid
  msg.position.x = mean_centroid.x / float(CAM_FRAME_WIDTH) * 2.0;  // Norm to half frame
  msg.position.y = mean_centroid.y / float(CAM_FRAME_HEIGHT) * 2.0;
  msg.position.z = centroids.size();  // Number of visible points

  // direction of centroids
  msg.yaw = Z;

  // OF velocity
  msg.velocity.x = of_mean_velocity.x;
  msg.velocity.y = of_mean_velocity.y;

  point_pub.publish(msg);


// DRAW VISUALS
#ifdef DT_BUILD_DEV
  // canny_output = canny_output * 0.2;

  // Draw centroid markers
  for (i = 1; i < centroids.size(); i++) {
    circle(frame, centroids[i], 4, cv::Scalar(100, 80, 50), -1, 8, 0);
    circle(frame, centroids[i], 4, cv::Scalar(100, 80, 50), 2, 8, 0);
    cv::Point2f linePoint(
      centroids[i].x + 50.0 * sin(contour_rects[i].angle*CV_PI/180.0f),   // sin(contour_angles[i]),
      centroids[i].y - 50.0 * cos(contour_rects[i].angle*CV_PI/180.0f));  // cos(contour_angles[i]));
    cv::arrowedLine(frame, centroids[i], linePoint,
      cv::Scalar(150, 90, 30), 2, 8, 0, 0.1);

    contour_rects[i].points(contour_rect_points);
    for (j = 0; j < 4; j++) {
      cv::line(frame,
        contour_rect_points[j], contour_rect_points[(j+1)%4],
        cv::Scalar(130, 70, 120));
    }
    // ROS_INFO_STREAM("Contour size: " << contour_rects[i].size);
  }

  // Draw average and bottom markers and heading line
  if (centroids.size()) {
    cv::Point2f midPoint(
      revertXCoord(mean_centroid.x),
      revertYCoord(mean_centroid.y));
    cv::Point2f hdgPoint(
      midPoint.x + CAM_VIS_LENGTH * sin(Z),
      midPoint.y - CAM_VIS_LENGTH * cos(Z));

    circle(frame, midPoint, 10, cv::Scalar(30, 200, 50), -1, 8, 0);
    circle(frame, centroids[0], 7, cv::Scalar(100, 150, 50), -1, 8, 0);
    circle(frame, centroids[0], 7, cv::Scalar(100, 150, 50), 2, 8, 0);
    cv::arrowedLine(frame, midPoint, hdgPoint, cv::Scalar(150, 50, 30), 2, 8, 0, 0.1);
  }

  // Draw mean velocity vector
  cv::arrowedLine(frame,
                  cv::Point(
                      CAM_FRAME_OFFSET_X,
                      CAM_FRAME_HEIGHT - CAM_FRAME_OFFSET_Y),
                  cv::Point(
                      CAM_FRAME_OFFSET_X + of_mean_velocity.x * CAM_FRAME_WIDTH,
                      CAM_FRAME_HEIGHT - (CAM_FRAME_OFFSET_Y + of_mean_velocity.y * CAM_FRAME_HEIGHT)),
                  cv::Scalar(40, 40, 200), 2, 8, 0, 0.1);

  // Draw old OF features
  for (i = 0; i < of_old_points.size(); ++i) {
    cv::drawMarker(
        frame, of_old_points[i], cv::Scalar(200, 30, 30),
        cv::MARKER_TILTED_CROSS, 5, 1, 8);
  }

  // Draw new OF features
  for (i = 0; i < of_new_points.size(); ++i) {
    cv::drawMarker(
        frame, of_new_points[i], cv::Scalar(50, 200, 30),
        cv::MARKER_TILTED_CROSS, 5, 1, 8);
  }

  mask3 = cv::Scalar(1, 1, 0);
  mask3.setTo(cv::Scalar(1, 1, 1), total_mask);
  cv::multiply(frame, mask3, frame);
  imshow(CAM_VIS_NAME, frame);

  cv::waitKey(1);

  #ifdef CAM_SHOW_FPS
    // Calculate and display fps
    float fps = 1.0f / (ros::Time::now().toSec() - startTime);  // (float) CLOCKS_PER_SEC; // Get FPS here
    ROS_INFO_STREAM("ros fps: " << fps);
  #endif
#endif

  // Update for next frame
  // dynamicMask();
  frame_gray_masked.copyTo(frame_gray_masked_old);
  frame_old_time = frame_time;
  of_old_points = of_new_points;
  ++of_refresh_counter;
}
