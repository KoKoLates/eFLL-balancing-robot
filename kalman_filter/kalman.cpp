# include "Arduino.h"
# include "kalman.h"

Kalman::Kalman(double angle, double bias, double measure){
  Q_Angle = angle;
  Q_Bias = bias;
  R_Measure = measure;
  K_Angle = 0, K_Bias = 0;

  for(int i=0; i<2; i++){
    for(int j=0; j<2; j++) p[i][j] = 0;
  }

  kt = (double)micros();
}

double Kalman::update_(double new_value, double new_rate){
  dt = (double)(micros() - kt) / 1e6;

  K_Rate = new_rate - K_Bias;
  K_Angle += K_Rate * dt;

  p[0][0] += (p[1][1] + p[0][1]+ Q_Angle ) * dt;
  p[0][1] -= p[1][1] * dt;
  p[1][0] -= p[1][1] * dt;
  p[1][1] += Q_Bias * dt;

  s = p[0][0] + R_Measure;
  k[0] = p[0][0] / s;
  k[1] = p[1][0] / s;

  y = new_value - K_Angle;

  K_Angle += k[0] * y;
  K_Bias += k[1] * y;

  p[0][0] -= k[0] * p[0][0];
  p[0][1] -= k[0] * p[0][1];
  p[1][0] -= k[1] * p[0][0];
  p[1][1] -= k[1] * p[0][1];

  kt = (double)micros();

  return K_Angle;
}
