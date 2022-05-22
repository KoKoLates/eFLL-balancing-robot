#include <Wire.h>
#include <helper_3dmath.h>
#include <MPU6050.h>
#include <I2Cdev.h>

#include <Fuzzy.h>

# define RMotor_DIG 4
# define RMotor_PWM 5
# define LMotor_DIG 6
# define LMotor_PWM 7

Fuzzy *fuzzy = new Fuzzy();
MPU6050 mpu;

int16_t acc_y, acc_z, gyro_x;

void fuzzy_set(){
  
}

void setup(){
  Serial.begin(9600);
  pinMode(RMotor_DIG, OUTPUT);
  pinMode(RMotor_PWM, OUTPUT);
  pinMode(LMotor_DIG, OUTPUT);
  pinMode(LMotor_PWM, OUTPUT);

  mpu.initialize();
  mpu.setYAccelOffset(175);
  mpu.setZAccelOffset(701);
  mpu.setXGyroOffset(97);
}

void loop(){
  acc_y = mpu.getAccelerationY();
  acc_z = mpu.getAccelerationZ();
  gyro_x = mpu.getRotationX();
  Serial.print(acc_y);
  Serial.print(" ");
  Serial.print(acc_z);
  Serial.print(" ");
  Serial.println(gyro_x);
  float accAngle = atan2(acc_y, acc_z)*RAD_TO_DEG;
  Serial.println(accAngle);
  delay(1000);
}
