#include <SoftPWM.h>
#include <EEPROM.h>
#include "Sensor.h"
#include "MotorDriver.h"
#include "PIDController.h"
#include <Encoder.h>
#include "FuzzyPIDGainScheduler.h"
#include "Kalman.h"
#include "Sonar.h"

#define ENCODER_PIN_AA  18
#define ENCODER_PIN_AB  19
#define ENCODER_PIN_BA  21
#define ENCODER_PIN_BB  20

Encoder encoderA(ENCODER_PIN_AA, ENCODER_PIN_AB);
Encoder encoderB(ENCODER_PIN_BA, ENCODER_PIN_BB);
double positionA = 0;
double positionB = 0;
double speedA = 0;
double speedB = 0;

#define MAIN_NUMP 5
#define MAIN_NUMD 3
SchedulerParams mainParams;
double mainPMF[MAIN_NUMP] = {-1,-0.3,0,0.3,1};
double mainDMF[MAIN_NUMD] = {-1,0,1};
double mainPRules[MAIN_NUMP*MAIN_NUMD] = {  2, 1.6, 1.2, 0.5,   0,
                                          1.5, 1.2,   1, 1.2, 1.5,
                                            0, 0.5, 1.2, 1.6,   2 };
double mainIRules[MAIN_NUMP*MAIN_NUMD] = {  1,   1,   1,   1,   1,
                                            1,   1,   1,   1,   1,
                                            1,   1,   1,   1,   1 };
double mainDRules[MAIN_NUMP*MAIN_NUMD] = {  1,   1,   1,   1,   1,
                                            1,   1,   1,   1,   1,
                                            1,   1,   1,   1,   1 };
FuzzyPIDGainScheduler mainS(&mainParams,MAIN_NUMP,MAIN_NUMD);

//void nvWrite(int ee, double value)
//{
//    byte* p = (byte*)(void*)&value;
//    for (int i = 0; i < sizeof(value); i++)
//        EEPROM.write(ee++, *p++);
//}
//
//double nvRead(int ee)
//{
//    double value = 0.0;
//    byte* p = (byte*)(void*)&value;
//    for (int i = 0; i < sizeof(value); i++)
//        *p++ = EEPROM.read(ee++);
//    return value;
//}

#define CENTER 0.0
PIDController mainPID(8,12,0.35);
PIDController speedPID(0.035,0.02,-0.00175);
PIDController positionPID(0.065,0.02,0.05);

#define STOPPED 1000
#define RUNNING 1001
#define AUTONOMOUS 1002

boolean balancing = true;
int state = STOPPED;
//boolean stopped = true;
boolean toStop = false;
#define TOSTOP_DELAY 3000
unsigned long toStopTimestamp = 0;

Kalman kalman;

void initialize()
{
  mainParams.pMF = mainPMF;
  mainParams.dMF = mainDMF;
  mainParams.pRules = mainPRules;
  mainParams.iRules = mainIRules;
  mainParams.dRules = mainDRules;
  mainParams.pInputGain = 5;
  mainParams.dInputGain = 2;
  mainParams.pOutputGain = 8;
  mainParams.iOutputGain = 8;
  mainParams.dOutputGain = 0.35;
  speedPID.setIntegralCut(1000);
  positionPID.setIntegralCut(5000);
}

#define SONARA_TRIGGER 12
#define SONARA_ECHO 10
#define SONARB_TRIGGER 13
#define SONARB_ECHO 11

unsigned int sonarARange = 0;
unsigned int sonarBRange = 0;

void sonarInitialize()
{
  pinMode(SONARA_TRIGGER,OUTPUT);
  pinMode(SONARB_TRIGGER,OUTPUT);
  pinMode(SONARA_ECHO,INPUT);
  pinMode(SONARB_ECHO,INPUT);
  digitalWrite(SONARA_TRIGGER,LOW);
  digitalWrite(SONARB_TRIGGER,LOW);
}

void sonarUpdate()
{
  digitalWrite(SONARA_TRIGGER,HIGH);
  delayMicroseconds(50);
  digitalWrite(SONARA_TRIGGER,LOW);
  sonarARange = pulseIn(SONARA_ECHO,HIGH,10000);
  if(sonarARange == 0)
    sonarARange = 3200;
  digitalWrite(SONARB_TRIGGER,HIGH);
  delayMicroseconds(50);
  digitalWrite(SONARB_TRIGGER,LOW);
  sonarBRange = pulseIn(SONARB_ECHO,HIGH,10000);
  if(sonarBRange == 0)
    sonarBRange = 3200;
}

#define LED_RED_PIN 4
#define LED_GREEN_PIN 3
#define LED_BLUE_PIN 2

void setup()
{
  Serial.begin(115200);
  sensorInitialize();
  motorInitialize();
  initialize();
  sonarInitialize();
  
  pinMode(LED_RED_PIN,OUTPUT);
  pinMode(LED_GREEN_PIN,OUTPUT);
  pinMode(LED_BLUE_PIN,OUTPUT);
  
  digitalWrite(LED_RED_PIN,LOW);
  digitalWrite(LED_GREEN_PIN,LOW);
  digitalWrite(LED_BLUE_PIN,LOW);
 
  
  delay(1000);
}

#define SENSOR_DELAY 2500
#define ENCODER_DELAY 50000
#define SONAR_DELAY 200000
#define SONAR_STOP_RANGE 1000
#define AUTONOMOUS_SPEED 3000
unsigned long sensorTimestamp = 0;
unsigned long encoderTimestamp = 0;
unsigned long sonarTimestamp = 0;
double gyroA = 0;

double speedC = 0;
double targetSpeedC = 0;
double positionC = 0;
double spid = 0;
double ppid = 0;
double leftC = 0;
double rightC = 0;
double motorOffset = 0;

void stopAndReset()
{
  setMotor(MOTOR_LEFT,0);
  setMotor(MOTOR_RIGHT,0);
  mainPID.reset();
  speedPID.reset();
  positionPID.reset();
  balancing = false;
  state = STOPPED;
  positionC = ( encoderA.read() + encoderB.read() ) / 2.0;

}

void loop()
{
  unsigned long timeMicros = micros();
  if((millis() - toStopTimestamp > TOSTOP_DELAY) && toStop)
  {
    positionC = (positionA+positionB)/2.0;
    state = STOPPED;
    toStop = false;
  }
  if(timeMicros - sensorTimestamp > SENSOR_DELAY)
  {
    double dt = (timeMicros - sensorTimestamp) / 1000000.0;
    double ang = angle();
//    Serial.println(ang);
//    double ang = kalman.getAngle(accel(),gyro(),dt);
    if(abs(ang)>PI/6)
    {
      stopAndReset();
    }
    if(balancing)
    {
//      mainS.compute(ang,CENTER + spid,kalman.getRate());
//      mainPID.setGains(mainS.Kp, mainS.Ki, mainS.Kd);
      double pid = mainPID.update(ang,CENTER + spid + ppid,gyro(),dt);
      setMotor(MOTOR_LEFT,pid + motorOffset);
      setMotor(MOTOR_RIGHT,pid - motorOffset);
    }
    else
    {
      if(abs(ang)<PI/32)
      {
        balancing = true;
        toStop = true;
        toStopTimestamp = millis();
        state = RUNNING;
        speedC = 0;
      }
    }
    sensorTimestamp = timeMicros;
  }
  if(timeMicros - encoderTimestamp > ENCODER_DELAY)
  {
//    setMotor(MOTOR_LEFT,0.25);
//    setMotor(MOTOR_RIGHT,0.25);
//    Serial.print(speedA);
//    Serial.print("\t");
//    Serial.println(speedB);
    long positionNA = encoderA.read();
    long positionNB = encoderB.read();
    double dt = (timeMicros - encoderTimestamp) / 1000000.0;
    double speedNA = ( positionNA - positionA ) / dt;
    double speedNB = ( positionNB - positionB ) / dt;
    double acc = (((speedNA+speedNB)/2.0) - ((speedA+speedB)/2.0))/dt;
    speedA = speedNA;
    speedB = speedNB;
    positionA = positionNA;
    positionB = positionNB;
    double ref = ((speedA+speedB)/2.0 )/17500;
    ref = max(-0.4,ref);
    ref = min(0.4,ref);
    double redV = 0.4 - abs(ref) * 2;
    redV = max(redV,0.0) * 2.5 * 255;
    analogWrite(LED_RED_PIN,(int)redV);
    double greenV = ref* 2.5 * 255;
    greenV = max(greenV,0.0);
    analogWrite(LED_GREEN_PIN,(int)greenV);
    double blueV = -ref* 2.5 * 255;
    blueV = max(blueV,0.0);
    analogWrite(LED_BLUE_PIN,(int)blueV);
    if(state == STOPPED)
    {
      spid = 0;
      ppid = positionPID.update((positionA+positionB)/2.0,positionC,(speedA+speedB)/2.0,dt) / 1000.0;
    }
    else
    {
      spid = speedPID.update((speedA+speedB)/2.0,speedC,acc,dt) / 1000.0;
      ppid = 0;
    }
    double alpha = 0.25;
    speedC = (1-alpha) * speedC + alpha * targetSpeedC;
    encoderTimestamp = timeMicros;
  }
  if(timeMicros - sonarTimestamp > SONAR_DELAY)
  {
    sonarUpdate();
//    state = AUTONOMOUS;
//    targetSpeedC = AUTONOMOUS_SPEED;
    if(state == RUNNING)
    {
      if(sonarARange < SONAR_STOP_RANGE || sonarBRange < SONAR_STOP_RANGE)
      {
        if(speedC > 0)
          targetSpeedC = 0;
      }
    }
    else if(state == AUTONOMOUS)
    {
      int leftS = (int)( sonarARange / 800.0 );
      int rightS = (int)( sonarBRange / 800.0 );
      if(leftS == 2 && rightS == 1)
      {
        targetSpeedC = AUTONOMOUS_SPEED * 0.5;
        motorOffset = 0.15;
      }
      else if(leftS == 1 && rightS == 2)
      {
        targetSpeedC = AUTONOMOUS_SPEED * 0.5;
        motorOffset = -0.15;
      }
      else if(leftS == 1 && rightS == 1)
      {
        targetSpeedC = 0;
        motorOffset = 0.25;
      }
      else if(leftS == 2 && rightS == 0)
      {
        targetSpeedC = 0;
        motorOffset = 0.25;
      }
      else if(leftS == 0 && rightS == 2)
      {
        targetSpeedC = 0;
        motorOffset = -0.25;
      }
      else 
      {
        targetSpeedC = AUTONOMOUS_SPEED;
        motorOffset = 0;
      }
    }
    sonarTimestamp = timeMicros;
//    Serial.print(sonarARange);
//    Serial.print("\t");
//    Serial.println(sonarBRange);
  }
}

#define BUFFER_SIZE 64
#define DELIMITER ';'
#define ACK '#'
#define COMMAND_HEARTBEAT 'A'
#define COMMAND_GETPARAMS 'B'
#define COMMAND_SETPARAMS 'C'
#define COMMAND_SETSPEED_LEFT 'D'
#define COMMAND_SETSPEED_RIGHT 'E'
#define COMMAND_POSITION_HOLD 'F'
#define COMMAND_AUTONOMOUS 'G'
char buffer[BUFFER_SIZE];
unsigned int bufferCount = 0;

void updateMove()
{
  targetSpeedC = ((leftC + rightC) / 2 ) * 5000;
  motorOffset = ( leftC - rightC ) * (1-(abs(targetSpeedC/10000.0))) * 0.1;
  if(state == RUNNING)
  {
    if(sonarARange < SONAR_STOP_RANGE || sonarBRange < SONAR_STOP_RANGE)
    {
      if(targetSpeedC > 0)
        targetSpeedC = 0;
    }
  }
  else
  {
    state == RUNNING;
  }
  if(state==STOPPED && abs(targetSpeedC) > 100)
  {
      state = RUNNING;
  }
}

void commandReceived()
{
  char buff[24];
  char cmd = buffer[0];
  if(cmd == COMMAND_HEARTBEAT)
  {
    Serial.write(ACK);
  }
//  if(cmd == COMMAND_GETPARAMS)
//  {
//    for(int i=2;i<bufferCount;i++)
//    {
//      buff[i-2] = buffer[i];
//    }
//    buff[bufferCount] = '\0';
//    int index = atoi(buff);
//    Serial.print(nvRead(sizeof(double)*index),4);
//    Serial.print(DELIMITER);
//  }
//  if(cmd == COMMAND_SETPARAMS)
//  {
//    int j = 2;
//    int i=0;
//    char c;
//    while(c != ':')
//    {
//      c = buffer[j++];
//      buff[i++] = c;
//    }
//    buff[i] = '\0';
//    int index = atoi(buff);
//    i = 0;
//    while(j < bufferCount)
//    {
//      buff[i++] = buffer[j++];
//    }
//    buff[i] = '\0';
//    double v = atof(buff);
//    nvWrite(sizeof(double)*index,v);
//  }
  if(cmd == COMMAND_SETSPEED_LEFT)
  {
    for(int i=2;i<bufferCount;i++)
    {
      buff[i-2] = buffer[i];
    }
    buff[bufferCount - 2] = '\0';
    leftC = atof(buff);
    updateMove();
  }
  if(cmd == COMMAND_SETSPEED_RIGHT)
  {
    for(int i=2;i<bufferCount;i++)
    {
      buff[i-2] = buffer[i];
    }
    buff[bufferCount - 2] = '\0';
    rightC = atof(buff);
    updateMove();
  }
  if(cmd == COMMAND_POSITION_HOLD)
  {
    state = STOPPED;
    positionC = ( encoderA.read() + encoderB.read() ) / 2.0;
  }
  if(cmd == COMMAND_AUTONOMOUS)
  {
    state = AUTONOMOUS;
    targetSpeedC = AUTONOMOUS_SPEED;
  }
}

void serialEvent() {
  while(Serial.available() > 0)
  {
    char c = Serial.read();
    if(c == DELIMITER)
    {
      commandReceived();
      bufferCount = 0;
    }
    else
    {
      buffer[bufferCount++] = c;
    }
  }
}


