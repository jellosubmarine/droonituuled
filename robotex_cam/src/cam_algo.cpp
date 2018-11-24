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

void Visuals::weighCentroids() {
  for (i = 0; i < centroids.size(); ++i) {
    weighted_centroids.push_back(cv::Point2f(
      // 0.5 * (centroids[i].x - CAM_FRAME_OFFSET_X) + CAM_FRAME_OFFSET_X,
      revertXCoord(
        sqrt(fabs(convertXCoord(centroids[i].x))) *
        sgn(convertXCoord(centroids[i].x)) * 10.0),
      centroids[i].y));
  }
}
