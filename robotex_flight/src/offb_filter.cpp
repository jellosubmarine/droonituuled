#include <cmath>
#include "offb_filter.hpp"

OffbFilter::OffbFilter() {
  dTimeConst = 0.0;
  dFilterGain = 0.0;
  Ts = 0.0;
  prevTime = 0.0;
  output = 0.0;
}

void OffbFilter::conf(double timeConst) {
  dTimeConst = timeConst;
}

void OffbFilter::initFirstInput(double input, double time) {
  output = input;
  prevTime = time;
}

double OffbFilter::filter(double input, double time) {
  Ts = time - prevTime;
  output = (1.0 - Ts / dTimeConst) * output + Ts / dTimeConst * input;
  prevTime = time;
  return output;
}
