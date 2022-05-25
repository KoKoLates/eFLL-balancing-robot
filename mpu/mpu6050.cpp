# include "mpu6050.h"
# include "Arduino.h"

# include "Wire.h"

MPU6050::MPU6050(int address)
{
    mpu = address;
}

void MPU6050::initial()
{
    Wire.begin();
    Wire.beginTransmission(mpu); 
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
    delay(20);
}

float MPU6050::accAngle()
{
    Wire.beginTransmission(mpu);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(mpu, 6, true);
  
    float acc_x = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0; 
    float acc_y = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0;
    float acc_z = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0;
    float acc_angle = (atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / PI);

    y = 0.95 * y1 + 0.025* acc_angle + 0.025 * x1;
    x1 = acc_angle;
    y1 = y;

    return y;
}

float MPU6050::gyroRate()
{ 
    Wire.beginTransmission(mpu);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(mpu, 6, true);
    float GyroX = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0; 
    float GyroY = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
    float GyroZ = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
    GyroX += 0.5;
    a = 0.95 * a1 + 0.025 * GyroX + 0.025 * b1;
    b1 = GyroX;
    a1 = a;

    return a;
}