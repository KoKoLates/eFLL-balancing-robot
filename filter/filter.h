# ifndef _FILTER_H__
# define _FILTER_H__

# include "Arduino.h"

// Kalman Filter
class Kalman{
  private:
    double Q_Angle, Q_Bias, R_Measure;
    double K_Angle, K_Bias, K_Rate;
    double p[2][2], k[2];
    double s, y, t, dt;

  public:
    Kalman(double angle = 0.001, double bias = 0.003, double measure = 0.03);
    double update_(double new_value, double new_rate); 
};

// Complementary Filter
class Complementary{
  private:
    float alpha;
    float pre_angle = 0, current_angle = 0;
    float time_step, current_time, pre_time;

  public:
    Complementary(float a = 0.95);
    float update_(float acc_Angle, float gyro_Angle);
};

// Low-Pass Filter
class LowPass{
  private:
    float alpha;
    float c[2][2];

  public:
    LowPass(float a = 0.95);
    float update_(float value);    
};

# endif
