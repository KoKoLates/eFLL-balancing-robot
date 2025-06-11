# include "mpu6050.h"
# include "Arduino.h"

MPU6050::MPU6050(int mpu_address)
{
    address = mpu_address;
    acc_error = gyro_error = 0;
}

void MPU6050::MPU6050_INIT(){
    // Initialize and Reset
    Wire.beginTransmission(address); 
    Wire.write(0x6B); // PWR_MGMT_1 Register
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Gyroscope Configuration
    Wire.beginTransmission(address); 
    Wire.write(0x1B); // Gyroscope Configuration Register
    Wire.write(0x00);
    // FS_SEL = 0, Full Scale Range = -/+ 250 degree/s
    // RAW_DATA * FULL_SCALE_RANGE / 32768.0 -> Gravity Values
    Wire.endTransmission(true);

    // Accelerometer Configuration
    Wire.beginTransmission(address); 
    Wire.write(0x1C); // Accelerometer Configuration Register
    Wire.write(0x00);
    // AFS_SEL = 0, Full Scale Range = -/+ 2g
    // RAW_DATA * FULL_SCALE_RANGE / 32768.0 -> Gravity Values
    Wire.endTransmission(true);
}

void MPU6050::MPU6050_CAL_(){
    // Calculating the error of MPU6050 
    int count = 0;
    while(count < 200){
        Wire.beginTransmission(address);
        // Starting from register 0x3B -> ACCEL_XOUT_H
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(address, 14, true);
        acc_x = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0;
        acc_y = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0;
        acc_z = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0;
        temp_ = (Wire.read() << 8 | Wire.read());
        gyro_x = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
        gyro_y = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
        gyro_z = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;

        // Summing the reading(pitch first)
        acc_error += atan((acc_y) / (sqrt(pow(acc_x, 2) + pow(acc_z, 2)))) * 180 / PI;
        gyro_error += gyro_x;
        count ++;
    }

    // Dividing by total number to get the error
    acc_error /= 200.0;
    gyro_error /= 200.0;
    count = 0;
}

void MPU6050::MPU6050_ACC(){
    Wire.beginTransmission(address);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(address, 6, true);
    acc_x = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0; 
    acc_y = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0;
    acc_z = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0;
}

void MPU6050::MPU6050_GYRO(){
    Wire.beginTransmission(address);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(address, 6, true);
    gyro_x = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
    gyro_y = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
    gyro_z = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
}

void MPU6050::MPU6050_DATA(){
    Wire.beginTransmission(address);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(address, 14, true);
    acc_x = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0;
    acc_y = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0;
    acc_z = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0;
    temp_ = (Wire.read() << 8 | Wire.read());
    gyro_x = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
    gyro_y = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
    gyro_z = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
}

float MPU6050::ACC_ANGLE(){
    // PITCH first, others TBA
    return atan(acc_y / (sqrt(pow(acc_x, 2) + pow(acc_z, 2)))) * 180 / PI - acc_error;
}

float MPU6050::GYRO_RATE(){
    // PITCH first, others TBA
    return (gyro_x - gyro_error);
}