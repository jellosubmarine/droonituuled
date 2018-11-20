#include <numeric>

#include "dt_config.hpp"
#include "cam.hpp"

using namespace std;
using namespace cv;

// Squared distance between two points
double dist2(const cv::Point2f &p1, const cv::Point2f &p2) {
  return (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);
}

float convertXCoord(float x) {
  return x - CAM_FRAME_OFFSET_X;
}

float convertYCoord(float y) {
  return CAM_FRAME_HEIGHT - y - CAM_FRAME_OFFSET_Y;
}

// Calculates the mean point of a given set
void mean_point(const std::vector<cv::Point2f> &points, cv::Point2f *output) {
  output->x = 0.0;
  output->y = 0.0;

  if (points.size() <= 0) return;

  for (int i = 0; i < points.size(); ++i) {
    output->x += points[i].x;
    output->y += points[i].y;
  }
  output->x /= float(points.size());
  output->y /= float(points.size());
  output->x = convertXCoord(output->x);
  output->y = convertYCoord(output->y);
}

// Function for covariance
float cov(const std::vector<float> &A, const std::vector<float> &B) {
  float n = A.size();
  float ma = accumulate(A.begin(), A.end(), 0.0) / A.size();
  float mb = accumulate(B.begin(), B.end(), 0.0) / B.size();
  float cov = 0;
  for (int i = 0; i < n; ++i) {
    cov += (A[i] - ma) * (B[i] - mb);
  }
  return cov * (1 / n);
}

float Visuals::hdgFromPca() {
    // PCA implementation
    c_x.clear();
    c_y.clear();
    for (int i = 0; i < centroids.size(); ++i) {  // To calculate cov, we need vectors with components
      c_x.push_back(centroids[i].x);
      c_y.push_back(CAM_FRAME_HEIGHT - centroids[i].y);
    }

    float XX = cov(c_x, c_x);  // Cov matrix elements
    float XY = cov(c_x, c_y);
    float YX = cov(c_y, c_x);
    float YY = cov(c_y, c_y);

    // We get 2 eigenvalues, we need the bigger one.
    float e_val1 = 0.5 * (XX + YY + sqrt(pow((XX + YY), 2) - 4 * (XX * YY - XY * YX)));  // first root
    float e_val2 = 0.5 * (XX + YY - sqrt(pow((XX + YY), 2) - 4 * (XX * YY - XY * YX)));  // second root
    float e_val = max(e_val1, e_val2);

    // Get the direction.
    // float hdg = atan2(1, XY / (e_val - XX)) - CV_PI / 2; // pi/2 subtracted to get values from -pi/2 to pi/2
    float hdg = atan2(XY/(e_val - XX), 1.0);
    // Note, that when drawing the direction vector, another pi/2 is subtracted to make the "0"
    // point upwards.
    return hdg;
}

// Heading calculation based on line fitting
float Visuals::hdgFromLineFit() {
  cv::fitLine(centroids, line_data, CV_DIST_L2, 0.0, 0.01, 0.01);
  float hdg = atan2(line_data[0], -line_data[1]);
  if (hdg > 2.09) hdg -= CV_PI;         // +120 deg
  else if (hdg < -2.09) hdg += CV_PI;   // -120 deg
  return hdg;
}

// Heading calculation based on bottom and middle points
float Visuals::hdgFromBottomPoint() {
  return atan2(mean_centroid.x - bottom_centroid.x,
               mean_centroid.y - bottom_centroid.y);
}

/* IMAGE PROCESSING */

void Visuals::process() {
  #ifdef CAM_SHOW_FPS
    // Start time for frame rate calculation
    float startTime = ros::Time::now().toSec();  // std::clock_t start( std::clock() );
  #endif


  // Color conversion and masking
  cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
  cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
  inRange(frame_HSV, Scalar(0, 0, 0), Scalar(255, rp_sat_thr, 255), hsv_mask);
  bitwise_and(hsv_mask, edge_mask, total_mask);
  bitwise_and(frame_gray, total_mask, frame_gray_masked);

  // Update optical flow tracked points at given interval
  if (of_refresh_counter >= rp_of_refresh_int) {
    of_refresh_counter = 0;
    of_old_points.clear();

    goodFeaturesToTrack(frame_gray_masked_old, of_old_points, CAM_GF_MAX_POINTS,
        CAM_GF_QUALITY, CAM_GF_MIN_DIST, cv::noArray(), CAM_GF_BLOCK_SIZE);

    ROS_INFO_STREAM("Found " << of_old_points.size() << " OF points");
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

  for (size_t i = 0; i < of_status.size(); ++i) {
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

  // Finished with OF

  // Process and find contours
  blur(frame_gray_masked, blurr, Size(CAM_BLUR_SIZE, CAM_BLUR_SIZE), Point(-1, -1));  // Blur image
  Canny(blurr, dil, CAM_CANNY_THR_LOW, CAM_CANNY_THR_HIGH, CAM_CANNY_KERNEL_SIZE);    // Find edges
  dilate(dil, canny_output, Mat(), Point(-1, -1), 4, 1, 1);                           // Get rid of holes
  // erode(dil, dil, Mat(), Point(-1, -1), 4, 1, 1);

  findContours(canny_output, contours, hierarchy,
    RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));  // Get contours


  // Clear storage vectors
  mu.clear();
  centroids.clear();

  // Calculate contour moments
  for (size_t i = 0; i < contours.size(); ++i) {  // Look through all contours
    if (contourArea(contours[i]) > CAM_CONTOUR_LIM_LOW &&
        contourArea(contours[i]) < CAM_CONTOUR_LIM_HIGH) {
      mu.push_back(moments(contours[i], false));
    }
  }

  // Calculate centroids from moments
  for (size_t i = 0; i < mu.size(); i++) {
    centroids.push_back(cv::Point2f(
        float(mu[i].m10) / float(mu[i].m00),
        float(mu[i].m01) / float(mu[i].m00)));
  }

  // Remove close lying centroids
  for (int i = 0; i < centroids.size(); ++i) {
    for (int j = i+1; j < centroids.size(); ++j) {
      if (dist2(centroids[i], centroids[j]) < CAM_CONTOUR_MIN_DIST2) {
        ROS_INFO("Removing duplicate centroid");
        centroids.erase(centroids.begin() + j);
        --j;
      }
    }
  }

  W = centroids.size();  // Assign here to avoid repeated size() calling
  if (W > 0) {
    // Find outliers using brute force
    distances = cv::Mat::zeros(W, W, CV_32F);
    float min_dist_abs = FLT_MAX;  // Min distance betw two centroids
    float min_dist_cur;            // Min distance for current centroid
    int i, j;

    // Determine minimum nearest neighbour distance
    for (i = 0; i < W; ++i) {
      for (j = i+1; j < W; ++j) {
        distances.at<float>(i, j) = dist2(centroids[i], centroids[j]);
        if (distances.at<float>(i, j) < min_dist_abs) {
          min_dist_abs = distances.at<float>(i, j);
        }
      }
    }

    // Find outliers
    for (i = 0; i < W; ++i) {
      min_dist_cur = FLT_MAX;

      // Compare to centroids before this one
      for (j = 0; j < i; ++j) {
        if (distances.at<float>(j, i) < min_dist_cur) {
          min_dist_cur = distances.at<float>(j, i);
        }
      }

      // Compare to centroids after this one
      for (j = i+1; j < W; ++j) {
        if (distances.at<float>(i, j) < min_dist_cur) {
          min_dist_cur = distances.at<float>(i, j);
        }
      }

      // Mark outliers for removal
      if (min_dist_cur > CAM_CONTOUR_OUTLIER * min_dist_abs) {
        centroids[i].x = FLT_MAX;
      }
    }

    // Remove outliers
    for (i = 0; i < centroids.size(); ++i) {
      if (centroids[i].x == FLT_MAX) {
        /*
        ROS_INFO_STREAM(
          "Dropped point [" <<
          centroids[i].x - CAM_FRAME_OFFSET_X <<
          ", " <<
          CAM_FRAME_HEIGHT - centroids[i].y - CAM_FRAME_OFFSET_Y <<
          "], min dist sq = " << largest_min_dist
        );
        */
        centroids.erase(centroids.begin() + i);
        --i;
      }
    }
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
  msg.position.z = W;  // Number of visible points

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
  for (size_t i = 1; i < W; i++) {
    circle(frame, centroids[i], 4, Scalar(100, 80, 50), 2, 8, 0);
  }

  // Draw average and bottom markers
  if (W > 0) {
    Point2f midPoint(
      static_cast<float>(mean_centroid.x + CAM_FRAME_OFFSET_X),
      static_cast<float>(CAM_FRAME_HEIGHT - mean_centroid.y - CAM_FRAME_OFFSET_Y));
    Point2f hdgPoint(
      // round(midPoint.x + CAM_VIS_LENGTH * cos(Z - CV_PI/2.0)),
      // round(midPoint.y + CAM_VIS_LENGTH * sin(Z - CV_PI/2.0))
      midPoint.x + CAM_VIS_LENGTH * sin(Z),
      midPoint.y - CAM_VIS_LENGTH * cos(Z));

      circle(frame, midPoint, 10, Scalar(30, 200, 50), -1, 8, 0);
      circle(frame, centroids[0], 7, Scalar(100, 80, 50), -1, 8, 0);
      cv::arrowedLine(frame, midPoint, hdgPoint, Scalar(150, 50, 30), 2, 8, 0, 0.1);
  }

  // Draw mean velocity vector
  cv::arrowedLine(frame,
                  cv::Point(
                      CAM_FRAME_OFFSET_X,
                      CAM_FRAME_HEIGHT - CAM_FRAME_OFFSET_Y),
                  cv::Point(
                      CAM_FRAME_OFFSET_X + of_mean_velocity.x * CAM_FRAME_WIDTH,
                      CAM_FRAME_HEIGHT - (CAM_FRAME_OFFSET_Y + of_mean_velocity.y * CAM_FRAME_HEIGHT)),
                  Scalar(40, 40, 200), 2, 8, 0, 0.1);

  // Draw old OF features
  for (int i = 0; i < of_old_points.size(); ++i) {
    cv::drawMarker(
        frame, of_old_points[i], Scalar(200, 30, 30),
        cv::MARKER_TILTED_CROSS, 5, 1, 8);
  }

  // Draw new OF features
  for (int i = 0; i < of_new_points.size(); ++i) {
    cv::drawMarker(
        frame, of_new_points[i], Scalar(50, 200, 30),
        cv::MARKER_TILTED_CROSS, 5, 1, 8);
  }

  mask3 = Scalar(1, 1, 0);
  mask3.setTo(Scalar(1, 1, 1), total_mask);
  cv::multiply(frame, mask3, frame);
  imshow(CAM_VIS_NAME, frame);

  waitKey(1);

  /*
      if (W==1){
        ROS_INFO("Msg Sent.");
        ROS_INFO("Subscribers: %d", point_pub.getNumSubscribers());
      }
    */

  #ifdef CAM_SHOW_FPS
    // Calculate and display fps
    float fps = 1.0f / (ros::Time::now().toSec() - startTime);  // (float) CLOCKS_PER_SEC; // Get FPS here
    cout << "ros fps: " << fps << '\n';
  #endif
#endif

  // Copy data for next iteration
  frame_gray_masked.copyTo(frame_gray_masked_old);
  frame_old_time = frame_time;
  of_old_points = of_new_points;
  ++of_refresh_counter;
}
