#include "offb_pid.hpp"
#include <cmath>


void OffbPID::conf(double P, double I, double D, double F, double DTC,
                double DK, double maxOut, double minOut, double outRamp) {
  this->p = P;
  this->i = I;
  this->d = D;
  this->f = F;
  dTimeConst = DTC;
  dFilterGain = DK;
  maxOutput = maxOut;
  minOutput = minOut;
  maxOutputRamp = outRamp;

  iOut = 0.0;
}

void OffbPID::initFirstInput(double input, double time) {
  dState = target - input;
  curInput = input;
  curTime = time;
}


double OffbPID::update(double input, double time) {
  curInput = input;
  prevErr = curErr;
  curErr = target - curInput;
  prevTime = curTime;
  curTime = time;

  if ( (output < maxOutput || curErr*i < 0) &&
       (output > minOutput || curErr*i > 0) )
  {
    integrate();
  }
  derivative();

  prevOutput = output;
  output = fmax(
      minOutput,
      fmin(maxOutput, p * curErr + i * iOut + d * dOut + f * target)
  );
  double outputPRamp = prevOutput + (curTime - prevTime) * maxOutputRamp;
  double outputNRamp = prevOutput + (prevTime - curTime) * maxOutputRamp;
  output = fmax( outputNRamp, fmin(outputPRamp, output) );

  return output;
}


void OffbPID::integrate() {
  iOut += (prevErr + curErr) / 2.0 * (curTime - prevTime);
}


void OffbPID::derivative() {
  dOut = dFilterGain / dTimeConst * (curErr - dState);
  double Ts = curTime - prevTime;
  dState = (1.0 - Ts / dTimeConst) * dState + Ts / dTimeConst * curErr;
}
