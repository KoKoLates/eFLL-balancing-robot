#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050.h>

MPU6050 mpu;
int16_t AccY, AccZ, GyroX;
float AccAngle, GyroAngle;
float yn=0, yn1=0, xn1=0;
float a=0, a1=0, b1=0;
float elapsedTime, currentTime, previousTime;

void setup() {
  Serial.begin(9600);
  mpu.initialize();
  mpu.setYAccelOffset(175);
  mpu.setZAccelOffset(701);
  mpu.setXGyroOffset(97);
}

void loop() {
  //read accelerometer data//
  AccY = mpu.getAccelerationY();
  AccZ = mpu.getAccelerationZ();
  //calculate the angle//
  AccAngle = atan2(AccY, AccZ)*RAD_TO_DEG;
  //low pass filter for accAngle
  yn = 0.92*yn1 + 0.04*AccAngle + 0.04*xn1;
  xn1=AccAngle;yn1=yn;
  
  //
  //read gyroscope data//
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  GyroX = mpu.getRotationX()/131;
  float gyroRate = GyroX;
  a = 0.95*a1 + 0.025*gyroRate + 0.025*b1;
  b1=gyroRate;a1=a;
  GyroAngle += gyroRate*elapsedTime;
  float angle = -0.01*GyroAngle + 1.01*yn;
  Serial.print(a);
  Serial.print(",");Serial.println(angle);
}#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050.h>

MPU6050 mpu;
int16_t acc_y, acc_z, gyro_x;
long sampling_timer;
float gyroAngle, currentangle;
double timeStep, Ttime, timePrev;
float xn1=0, yn1=0, yn=0;
float a1=0, b1=0, b=0;
float preangle=0;

void setup() {
  Serial.begin(9600);
  mpu.initialize();
  mpu.setYAccelOffset(175);
  mpu.setZAccelOffset(701);
  mpu.setXGyroOffset(97);
  Ttime = millis();
}

void loop() {
  timePrev = Ttime;
  preangle = yn;
  acc_y = mpu.getAccelerationY();
  acc_z = mpu.getAccelerationZ();
  gyro_x = mpu.getRotationX();
  float accAngle = atan2(acc_y, acc_z)*RAD_TO_DEG;
  yn = 0.9*yn1 + 0.05*accAngle + 0.05*xn1;
  yn1 = yn;
  xn1 = accAngle;
//  float gyroRate = map(gyro_x, -32768, 32767, -250, 250);
  float dangle = (yn-preangle)/timeStep;
  Serial.print(accAngle);
  Serial.print(" , ");
  Serial.print(dangle);
  Serial.print(", ");
  Serial.println(yn);
  Ttime = millis();
  timeStep = (Ttime - timePrev)/1000;
}
