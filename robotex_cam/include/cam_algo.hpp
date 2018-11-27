#ifndef DT_CAM_ALGO_INCLUDED
#define DT_CAM_ALGO_INCLUDED

#include <vector>
#include "opencv2/core.hpp"

// template <typename T> int sgn(T val);

float convertXCoord(float x);
float revertXCoord(float x);
float convertYCoord(float y);
float revertYCoord(float y);

double dist2(const cv::Point2f &p1, const cv::Point2f &p2);
void mean_point(const std::vector<cv::Point2f> &points, cv::Point2f *output);

#endif
