#ifndef OFFB_FILTER_INCLUDED
#define OFFB_FILTER_INCLUDED

class OffbFilter {
public:
  OffbFilter();
  void conf(double timmeConst);
  void initFirstInput(double input, double time);
  double filter(double input, double time);

  // Filter constants
  double dTimeConst;
  double dFilterGain;
  double output;

private:
  // Filter persistent states
  double prevTime, Ts;
};

#endif
