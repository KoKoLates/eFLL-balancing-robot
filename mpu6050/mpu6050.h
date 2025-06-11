# ifndef _MPU6050_H_
# define _MPU6050_H_

# include "Arduino.h"
# include "Wire.h"

class MPU6050{
    private:
        // I2C address of MPU6050
        int address;

        // Raw Data of MPU6050
        float acc_x, acc_y, acc_z, temp_;
        float gyro_x, gyro_y, gyro_z;
        float acc_error, gyro_error;

    public:
        // Initialize Configuration
        MPU6050(int);
        void MPU6050_INIT();
        void MPU6050_CAL_();

        // Obtain Data
        void MPU6050_ACC();
        void MPU6050_GYRO();
        void MPU6050_DATA();

        float ACC_ANGLE();
        float GYRO_RATE();
};

# endif