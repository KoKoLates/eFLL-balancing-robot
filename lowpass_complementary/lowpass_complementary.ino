#include <Wire.h>
const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, GyroErrorX;
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
  calculate_IMU_error();
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
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
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
  Wire.requestFrom(MPU, 2, true);
  GyroX = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0 - GyroErrorX;
  // Correct the outputs with the calculated error values
  b = 0.95*b1 + 0.025*GyroX + 0.025*a1;
  b1 = b; a1 = GyroX;
//  Serial.print(yn);
//  Serial.print(" ,");
  Serial.print(b);
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX += GyroX * elapsedTime;
//  Serial.print(" ,");
//  Serial.print(gyroAngleX);
  
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.1 * gyroAngleX + 0.9 * yn;
  Serial.print(" ,");
  Serial.println(roll);
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
//    GyroErrorY = GyroErrorY + (GyroY / 131.0);
//    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  // Print the error values on the Serial Monitor
//  Serial.print("AccErrorX: ");
//  Serial.println(AccErrorX);
//  Serial.print("GyroErrorX: ");
//  Serial.println(GyroErrorX);
}
