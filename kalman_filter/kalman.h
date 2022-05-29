# ifndef _KALMAN_H__
# define _KALMAN_H__

# include "Arduino.h"

class Kalman{
  private:
    double Q_Angle, Q_Bias, R_Measure;
    double K_Angle, K_Bias, K_Rate;
    double p[2][2], k[2];
    double s, y;
    double dt, kt;

  public:
    Kalman(double angle = 0.001, double bias = 0.003, double measure = 0.03);
    double update_(double new_value, double new_rate);
};

# endif
