#include "Arduino.h"
#include "QuickSelect.h"

#define GYRO_RATE_PIN A1
#define GYRO_TEMP_PIN A0
#define GYRO_CENTER 484.5
#define GYRO_SCALE 0.015
#define GYRO_TEMP_SCALE 1.0

#define ACCEL_SPI_CLK A2
#define ACCEL_SPI_MISO A3
#define ACCEL_SPI_MOSI A4
#define ACCEL_SPI_CS A5

#define ACCEL_X_BIAS 42
#define ACCEL_Y_BIAS 0
#define ACCEL_Y_MAX 270

byte sensorFirstMeasure;
long lastTime;
double lAngle;
double gyroBias;

byte spiOut(byte spiDat)
{
  byte bitNum=8;
  byte spiRead=0;
  // start spi bit bang
  while(bitNum>0){
    pinMode(ACCEL_SPI_CLK,OUTPUT); // SPI CLK =0
    if((spiDat & 0x80)!=0)
      pinMode(ACCEL_SPI_MOSI,INPUT); // MOSI = 1 if MSB =1
    else
      pinMode(ACCEL_SPI_MOSI,OUTPUT); // else MOSI = 0
    spiDat=spiDat<<1; 
    pinMode(ACCEL_SPI_CLK,INPUT); // SPI CLK = 1
    // read spi data
    spiRead=spiRead<<1;
    if(digitalRead(ACCEL_SPI_MISO)==HIGH) spiRead |= 0x01;
    // shift in a 1 if MISO is 1
    pinMode(ACCEL_SPI_MOSI,INPUT); // reset MOSI to 1
    bitNum--; 
  }
  return spiRead;
}

void initADXL(void)
{
  delay(250);
  pinMode(ACCEL_SPI_CS,OUTPUT);
  spiOut(0x31);
  spiOut(0x08);
  pinMode(ACCEL_SPI_CS,INPUT);
  delay(1);
  pinMode(ACCEL_SPI_CS,OUTPUT);
  spiOut(0x2d);
  spiOut(0x08);
  pinMode(ACCEL_SPI_CS,INPUT);
  delay(1);
}

#define NUM_ACCUM 9
int accelAccum[2][NUM_ACCUM]; 

void sensorInitialize()
{
  initADXL();
  byte sensorFirstMeasure = 1;
  for(int j=0;j<NUM_ACCUM;j++)
  {
    accelAccum[0][j] = 0;
    accelAccum[1][j] = ACCEL_Y_MAX;
  }
}

double accel()
{
  int i;
  byte xyz[8];
  double angle = 0;
  int accel[3];
  pinMode(ACCEL_SPI_CS,OUTPUT);
  spiOut(0xF2);
  for(i=0;i<6;i++){
    xyz[i]=spiOut(0x00);
  }
  accel[0]=((int)xyz[1]<<8) + xyz[0];
  accel[1]=((int)xyz[3]<<8) + xyz[2];
  accel[2]=((int)xyz[5]<<8) + xyz[4];
  pinMode(ACCEL_SPI_CS,INPUT);
  int x = accel[2] + ACCEL_X_BIAS;
  int y = accel[1] + ACCEL_Y_BIAS;
  for(int i=0;i<NUM_ACCUM-1;i++)
  {
    accelAccum[0][i] = accelAccum[0][i+1];
    accelAccum[1][i] = accelAccum[1][i+1];
  }
  accelAccum[0][NUM_ACCUM-1] = x;
  accelAccum[1][NUM_ACCUM-1] = y;
  
  int accum[2][NUM_ACCUM];
    for(int i=0;i<NUM_ACCUM;i++)
  {
    accum[0][i] = accelAccum[0][i];
    accum[1][i] = accelAccum[1][i];
  }
  
  x = quick_select(accum[0],NUM_ACCUM/2);
  y = quick_select(accum[1],NUM_ACCUM/2);
  
  angle = atan((x*1.0)/y);
  return -angle;
}

double gyro()
{
  int gyroM = analogRead(GYRO_RATE_PIN)-GYRO_CENTER;
  return gyroM*GYRO_SCALE;
}

double gyroTemp()
{
  return analogRead(GYRO_TEMP_PIN)*GYRO_TEMP_SCALE;
}

#define TAU 5

float angle() {
  if(sensorFirstMeasure)
  {
    sensorFirstMeasure = 0;
    lAngle = accel();
    lastTime = micros();
    return lAngle;
  }
  else
  {
    double accelV = accel();
    double gyroV = gyro();
    long thisTime = micros();
    double dt = (thisTime - lastTime)/1000000.0;
    double alpha = TAU/(TAU+dt);
    double angle = alpha * (lAngle + gyroV * dt) + (1-alpha) * (accelV);
    lAngle = angle;
    lastTime = thisTime;
    return angle;
  }
}
