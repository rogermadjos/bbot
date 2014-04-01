#ifndef pidcontroller_h
#define pidcontroller_h
#include <Arduino.h>

class PIDController
{
  public:
    PIDController(double,double,double);
    void setLimits(double,double);
    double update(double,double,double,double);
    void setGains(double,double,double);
    void setIntegralCut(double);
    void reset();
  private:
    double Kp;
    double Kd;
    double Ki;
    double limLow;
    double limHigh;
    double integral;
    double integralCut;
};
#endif
