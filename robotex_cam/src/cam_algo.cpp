#include <numeric>
#include <vector>

#include "dt_config.hpp"
#include "cam.hpp"
#include "cam_algo.hpp"


// Signum
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


// Converts x-coordinate from camera to flight frame
float convertXCoord(float x) {
  return x - CAM_FRAME_OFFSET_X;
}
float revertXCoord(float x) {
  return x + CAM_FRAME_OFFSET_X;
}

// Converts y-coordinate from camera to flight frame
float convertYCoord(float y) {
  return CAM_FRAME_HEIGHT - y - CAM_FRAME_OFFSET_Y;
}
float revertYCoord(float y) {
  return CAM_FRAME_HEIGHT - y - CAM_FRAME_OFFSET_Y;
}

// Squared distance between two points
double dist2(const cv::Point2f &p1, const cv::Point2f &p2) {
  return (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);
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
  return cov * (1.0f / n);
}


/* Functions belonging to Visuals */

// Applies weighing to centroid X coordinates.
/*
void Visuals::weighCentroids() {
  // Uncomment to disable
  // weighted_centroids = centroids;
  // return;

  for (i = 0; i < centroids.size(); ++i) {
    // Div by const
    // 0.5 * (centroids[i].x - CAM_FRAME_OFFSET_X) + CAM_FRAME_OFFSET_X,

    // Sqrt
    weighted_centroids.push_back(cv::Point2f(
      revertXCoord(
        sqrt(fabs(convertXCoord(centroids[i].x))) *
        sgn(convertXCoord(centroids[i].x)) * 10.0),
      centroids[i].y));
  }
}
*/

// Creates a frame mask based on the number of contours
void Visuals::dynamicMask() {
  edge_mask = cv::Mat::zeros(frame.size(), CV_8UC1) + cv::Scalar(255);

  // Use full frame, if too few contours are found
  if (centroids.size() <= rp_dynmask_req_contours) return;

  // Todo: implement gradual narrowing
  dynmask_width = rp_dynmask_max_width;

  edge_mask(cv::Rect(0, 0,
    dynmask_width, CAM_FRAME_HEIGHT)) = cv::Scalar(0);
  edge_mask(cv::Rect(CAM_FRAME_WIDTH - dynmask_width, 0,
    dynmask_width, CAM_FRAME_HEIGHT)) = cv::Scalar(0);
}



/* Target heading calculation */

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
    float e_val = std::max(e_val1, e_val2);

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


// Estimates contour orientation
float contour_angle(const cv::Moments& m) {
  float mu20p = m.mu20 / m.m00;
  float mu02p = m.mu02 / m.m00;
  float mu11p = m.mu11 / m.m00;
  return 0.5f * atan2(-2.0f * mu11p, mu02p - mu20p );
}
