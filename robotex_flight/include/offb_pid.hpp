#ifndef OFFB_PID_INCLUDED
#define OFFB_PID_INCLUDED

class OffbPID {
public:
  void update(double input, double vel, double time);
  void update(double input, double time);
  void initFirstInput(double input, double time);
  void conf(double P, double I, double D, double F, double DTC, double DK,
            double outBias, double maxOut, double minOut, double outRamp);

  // PID gains
  double p;
  double i;
  double d;
  double f;

  // Derivative filter constants
  double dTimeConst;
  double dFilterGain;

  // Output configuration
  double target;
  double maxOutput;
  double minOutput;
  double maxOutputRamp;
  double bias;

  // Controller output
  double prevOutput;
  double output;

//private:
  void integrate();
  void derivative();
  void updateInput(double input, double time);
  void updateOutput();

  // Persistent states for integral and derivative
  double iOut;
  double dOut;
  double dState;

  // Controller persistent states
  double curInput;
  double curTime;
  double prevTime;
  double curErr;
  double prevErr;
};

#endif
