# ifndef _MPU6050_H_
# define _MPU6050_H_

# include "Arduino.h"

class MPU6050{
    private:
        int mpu;
        float _accAngle = 0, a1 = 0, a2 = 0;
        float _gyroRate = 0, b1 = 0, b2 = 0;
        float Time_step, current_time, previous_time;

    public:
        MPU6050(int);
        void initialize();
        float getAccAngle();
        float getGyroRate();
        float getGyroAngle();
};

# endif