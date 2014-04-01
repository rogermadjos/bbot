#include "PIDController.h"

PIDController::PIDController(double Kp,double Ki,double Kd) 
{
  PIDController::Kp = Kp;
  PIDController::Ki = Ki;
  PIDController::Kd = Kd;
  PIDController::setLimits(-255, 255);
  integralCut = 1000000;
}

void PIDController::setIntegralCut(double cut)
{
  integralCut = cut;
}

void PIDController::reset()
{
  integral = 0;
}

void PIDController::setLimits(double low, double high)
{
  limLow = low;
  limHigh = high;
}

double PIDController::update(double actual, double desired, double rate, double dt)
{
  double error = desired - actual;
  integral += error*dt;
  integral = min(integral,integralCut);
  integral = max(integral,-integralCut);
  double derivative = rate;
  double output = Kp * error + Ki * integral - Kd * derivative;
  output = min(output,limHigh);
  output = max(output,limLow);
  return output;
}

void PIDController::setGains(double Kp,double Ki,double Kd)
{
//  if(Kp<0 || Ki<0 || Kd<0)
//  {
//    return;
//  }
  PIDController::Kp = Kp;
  PIDController::Ki = Ki;
  PIDController::Kd = Kd;
}


