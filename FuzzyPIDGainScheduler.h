#ifndef fuzzypidgainscheduler_h
#define fuzzypidgainscheduler_h

#include <Arduino.h>

typedef struct SchedulerParams
{
  double * pMF;
  double * dMF;
  double * pRules;
  double * iRules;
  double * dRules;
  double pInputGain;
  double dInputGain;
  double pOutputGain;
  double iOutputGain;
  double dOutputGain;
};

class FuzzyPIDGainScheduler
{
  private:
    unsigned int numP;
    unsigned int numD;
    SchedulerParams * params;
    double activationP[7];
    double activationD[7];
    double gaussian(double x, double c, double s)
    {
      double diff = x - c;
      return exp(-((diff*diff)/(2*s*s)));
    }
  public:
    FuzzyPIDGainScheduler(SchedulerParams * params, unsigned int numP,unsigned int numD)
    {
      FuzzyPIDGainScheduler::params = params;
      FuzzyPIDGainScheduler::numP = numP;
      FuzzyPIDGainScheduler::numD = numD;
    }
    void compute(double actual,double desired, double rate)
    {
      double P = actual - desired;
      double D = rate;
      
      double total = 0;
      double pGain = 0;
      double iGain = 0;
      double dGain = 0;
      
      P *= params->pInputGain;
      D *= params->dInputGain;
      
//      Serial.print(P,4);
//      Serial.print("  ");
//      Serial.println(D,4);
      
      for(int i=0;i<numP;i++)
      {
        if(i==0)
        {
          if(P < params->pMF[i])
          {
            activationP[i] = 1;
          }
          else
          {
            double s = (params->pMF[i+1] - params->pMF[i])/3.0;
            activationP[i] = gaussian(P,params->pMF[i],s);
          }
        }
        else if(i==numP-1)
        {
          if(P > params->pMF[i])
          {
            activationP[i] = 1;
          }
          else
          {
            double s = (params->pMF[i] - params->pMF[i-1])/3.0;
            activationP[i] = gaussian(P,params->pMF[i],s);
          }
        }
        else
        {
          double s = 0;
          if(P > params->pMF[i])
          {
            s = (params->pMF[i+1] - params->pMF[i])/3.0;
          }
          else
          {
            s = (params->pMF[i] - params->pMF[i-1])/3.0;
          }
          activationP[i] = gaussian(P,params->pMF[i],s);
        }
      }
      
//      for(int i=0;i<5;i++)
//      {
//        Serial.print(activationP[i],5);
//        Serial.print("\t");
//      }
//      Serial.println();
      
      for(int i=0;i<numD;i++)
      {
        if(i==0)
        {
          if(D < params->dMF[i])
          {
            activationD[i] = 1;
          }
          else
          {
            double s = (params->dMF[i+1] - params->dMF[i])/3.0;
            activationD[i] = gaussian(D,params->dMF[i],s);
          }
        }
        else if(i==numD-1)
        {
          if(D > params->dMF[i])
          {
            activationD[i] = 1;
          }
          else
          {
            double s = (params->dMF[i] - params->dMF[i-1])/3.0;
            activationD[i] = gaussian(D,params->dMF[i],s);
          }
        }
        else
        {
          double s = 0;
          if(D > params->dMF[i])
          {
            s = (params->dMF[i+1] - params->dMF[i])/3.0;
          }
          else
          {
            s = (params->dMF[i] - params->dMF[i-1])/3.0;
          }
          activationD[i] = gaussian(D,params->dMF[i],s);
        }
      }
      
      for(int j=0;j<numD;j++)
      {
        for(int i=0;i<numP;i++)
        {
          double activation = activationP[i] * activationD[j];
          total += activation;
          pGain += params->pRules[j*numP + i] * activation;
          iGain += params->iRules[j*numP + i] * activation;
          dGain += params->dRules[j*numP + i] * activation;
        }
      }
      
      FuzzyPIDGainScheduler::Kp = ( pGain / total ) * params->pOutputGain;
      FuzzyPIDGainScheduler::Ki = ( iGain / total ) * params->iOutputGain;
      FuzzyPIDGainScheduler::Kd = ( dGain / total ) * params->dOutputGain;
    }
    double Kp;
    double Ki;
    double Kd;
};

#endif
