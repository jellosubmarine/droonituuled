#include "offb_pid.hpp"
#include <cmath>

OffbPID::OffbPID() {
  p = 0.0;
  i = 0.0;
  d = 0.0;
  f = 0.0;
  dTimeConst = 0.0;
  dFilterGain = 0.0;
  target = 0.0;
  maxOutput = 0.0;
  minOutput = 0.0;
  maxOutputRamp = 0.0;
  bias = 0.0;
  prevOutput = 0.0;
  output = 0.0;
  iOut = 0.0;
  dOut = 0.0;
  dState = 0.0;
  curInput = 0.0;
  curTime = 0.0;
  prevTime = 0.0;
  curErr = 0.0;
  prevErr = 0.0;
  outputPRamp = 0.0;
  outputNRamp = 0.0;
}

void OffbPID::conf(double P, double I, double D, double F, double DTC,
                double DK, double outBias,
                double maxOut, double minOut, double outRamp) {
  this->p = P;
  this->i = I;
  this->d = D;
  this->f = F;
  this->bias = outBias;
  dTimeConst = DTC;
  dFilterGain = DK;
  maxOutput = maxOut;
  minOutput = minOut;
  maxOutputRamp = outRamp;
}

void OffbPID::initFirstInput(double input, double time) {
  curInput = input;
  curTime = time;
  prevTime = curTime;
  dState = target - curInput;
  prevOutput = 0.0;
  output = 0.0;
  iOut = 0.0;
  dOut = 0.0;
}


void OffbPID::update(double input, double time) {
  updateInput(input, time);
  derivative();
  updateOutput();
}

void OffbPID::update(double input, double vel, double time) {
  updateInput(input, time);
  dOut = vel;
  updateOutput();
}

void OffbPID::updateInput(double input, double time) {
  curInput = input;
  prevErr = curErr;
  curErr = target - curInput;
  prevTime = curTime;
  curTime = time;
}

void OffbPID::updateOutput() {
  if ((output < maxOutput || curErr*i < 0) &&
      (output > minOutput || curErr*i > 0)) {
    integrate();
  }

  prevOutput = output;
  output = fmax(minOutput,
    fmin(maxOutput, p * curErr + i * iOut + d * dOut + f * target + bias));
  outputPRamp = prevOutput + (curTime - prevTime) * maxOutputRamp;
  outputNRamp = prevOutput + (prevTime - curTime) * maxOutputRamp;
  output = fmax(outputNRamp, fmin(outputPRamp, output));
}


void OffbPID::integrate() {
  iOut += (prevErr + curErr) / 2.0 * (curTime - prevTime);
}


void OffbPID::derivative() {
  dOut = dFilterGain / dTimeConst * (curErr - dState);
  Ts = curTime - prevTime;
  dState = (1.0 - Ts / dTimeConst) * dState + Ts / dTimeConst * curErr;
}
