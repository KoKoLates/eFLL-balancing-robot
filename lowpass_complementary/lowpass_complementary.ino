#include <Wire.h>
const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;
int c = 0;

float yn=0, yn1=0, xn1=0;
float a1=0, b=0, b1=0;
void setup() {
  Serial.begin(19200);
  Wire.begin();
  Wire.beginTransmission(MPU); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(20);
}
void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  
  AccX = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0; 
  AccY = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0;
  AccZ = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0;
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
  yn = 0.95*yn1 + 0.025*accAngleX + 0.025*xn1;
  xn1 = accAngleX;
  yn1 = yn;

  //  Serial.print(yn);
  //  Serial.print(" ," );
  //  Serial.println(accAngleX);
  // === Read gyroscope data === //
  previousTime = currentTime; 
  currentTime = millis(); 
  elapsedTime = (currentTime - previousTime) / 1000; 
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0; 
  GyroY = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
  // Correct the outputs with the calculated error values
  float gyroRate = GyroX + 0.5;
  b = 0.95*b1 + 0.025*GyroX + 0.025*a1;
  b1 = b; a1 = GyroX;
  Serial.print(yn);
  Serial.print(" ,");
  Serial.print(b);
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime;
  Serial.print(" ,");
  Serial.print(gyroAngleX);
  
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.1 * gyroAngleX + 0.9 * yn;
  Serial.print(" ,");
  Serial.println(roll);
}
