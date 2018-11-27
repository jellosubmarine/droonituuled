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


/* Functions belonging to Visuals */

// Heading calculation based on bottom and middle points
float Visuals::hdgFromBottomPoint() {
  return atan2(mean_centroid.x - bottom_centroid.x,
               mean_centroid.y - bottom_centroid.y);
}
