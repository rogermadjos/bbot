#ifndef sonar_h
#define sonar_h
#include <Arduino.h>

class Sonar
{
  public:
    Sonar(int trigger, int echo)
    {
      triggerPin = trigger;
      echoPin = echo;
      pinMode(triggerPin,OUTPUT);
      pinMode(echoPin,INPUT);
      attachInterrupt(echoPin,reinterpret_cast<void (*)()>(&interruptF),RISING);
      digitalWrite(triggerPin,LOW);
      range = 0;
    }
    void update()
    {
      timestamp = micros();
      digitalWrite(triggerPin,HIGH);
      delayMicroseconds(10);
      digitalWrite(triggerPin,LOW);
    }
    int getRange() {return range;}
  private:
    static void interruptF()
    {
      range = micros() - timestamp;
    };
    static unsigned int timestamp;
    int triggerPin;
    int echoPin;
    static unsigned int range;
};

#endif
