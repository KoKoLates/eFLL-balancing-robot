# ifndef _MPU6050_H_
# define _MPU6050_H_

# include "Arduino.h"

class MPU6050{
    private:
        int mpu;
        float y = 0, y1 = 0, x1 = 0;
        float a = 0, a1 = 0, b1 = 0;
        float elapsedTime, currentTime, previousTime;

    public:
        MPU6050(int);
        void initial();
        float accAngle();
        float gyroRate();
};

# endif