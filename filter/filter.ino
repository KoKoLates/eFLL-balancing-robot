# include <mpu6050.h>
# include "filter.h"

MPU6050 mpu(0x68);
LowPass low_pass_acc(0.8);
LowPass low_pass_gyro(0.8);
Complementary CF(0.95);
Kalman KF(0.001, 0.003, 0.03);

float accAngle, gyroRate;
float angle_k, angle_c, angle_l, gyro_l;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.MPU6050_INIT();
  mpu.MPU6050_CAL_();
}

void loop() {
  mpu.MPU6050_ACC();

  accAngle = mpu.ACC_ANGLE();
  gyroRate = mpu.GYRO_RATE();
  

  angle_l = low_pass_acc.update_(accAngle);
  gyro_l = low_pass_acc.update_(gyroRate);

  angle_c = CF.update_(angle_l, gyro_l);
  angle_k = KF.update_(angle_l, gyro_l);

  Serial.print(accAngle);
  Serial.print(" ,");
  Serial.print(angle_l);
  Serial.print(" ,");
  Serial.print(angle_c);
  Serial.print(" ,");
  Serial.println(angle_k);
}
