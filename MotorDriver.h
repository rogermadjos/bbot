#include <Arduino.h>

#define MOTOR_PIN_LEFT_A  7
#define MOTOR_PIN_LEFT_B  6
#define MOTOR_PIN_RIGHT_A 8
#define MOTOR_PIN_RIGHT_B 9
#define MOTOR_LEFT   1
#define MOTOR_RIGHT  2

#define MOTOR_LEFT_SCALE_POS 1
#define MOTOR_LEFT_SCALE_NEG 1.15
#define MOTOR_RIGHT_SCALE_POS 1.02
#define MOTOR_RIGHT_SCALE_NEG 1

void motorInitialize()
{
  pinMode(MOTOR_PIN_LEFT_A,OUTPUT);
  pinMode(MOTOR_PIN_LEFT_B,OUTPUT);
  pinMode(MOTOR_PIN_RIGHT_A,OUTPUT);
  pinMode(MOTOR_PIN_RIGHT_B,OUTPUT);

  digitalWrite(MOTOR_PIN_LEFT_A,LOW);
  digitalWrite(MOTOR_PIN_LEFT_B,LOW);
  digitalWrite(MOTOR_PIN_RIGHT_A,LOW);
  digitalWrite(MOTOR_PIN_RIGHT_B,LOW);
}

void setMotor(int motor, float value) {
  if(motor == MOTOR_LEFT)
  {
    if(value >= 0)
    {
      value *= MOTOR_LEFT_SCALE_POS;
    }
    else
    {
      value *= MOTOR_LEFT_SCALE_NEG;
    }
  }
  else if(motor == MOTOR_RIGHT)
  {
    if(value >= 0)
    {
      value *= MOTOR_RIGHT_SCALE_POS;
    }
    else
    {
      value *= MOTOR_RIGHT_SCALE_NEG;
    }
  }
  if(value > 1)
    value = 1;
  if(value < -1)
    value = -1;
  int pinA = MOTOR_PIN_LEFT_A;
  int pinB = MOTOR_PIN_LEFT_B;
  if(motor == MOTOR_RIGHT) {
    pinA = MOTOR_PIN_RIGHT_A;
    pinB = MOTOR_PIN_RIGHT_B;
  }
  int mag = round((abs(value) * 255));
  if(value < 0) {
    digitalWrite(pinB,LOW);
    analogWrite(pinA,mag);
  }
  else {
    digitalWrite(pinA,LOW);
    analogWrite(pinB,mag);
  }
  
}

