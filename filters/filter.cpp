# include "filter.h"
# include "Arduino.h"

// Kalman Filter Defination//
Kalman::Kalman(double angle, double bias, double measure){
  Q_Angle = angle;
  Q_Bias = bias;
  R_Measure = measure;
  K_Angle = 0, K_Bias = 0;

  for(int i=0; i<2; i++){
    for(int j=0; j<2; j++) p[i][j] = 0;
  }

  t = (double)micros();
}

double Kalman::update_(double new_value, double new_rate){
  dt = (double)(micros() - t) / 1e6;

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

  t = (double)micros();

  return K_Angle;
}


// Complementary Filter Defination
Complementary::Complementary(float a){
  alpha = a;
  time_step = current_time = pre_time = 0;
}

float Complementary::update_(float acc_Angle, float gyro_Rate){
  pre_time = current_time; 
  current_time = millis(); 
  time_step = (current_time - pre_time) / 1000;
  current_angle = alpha * (gyro_Rate * time_step + pre_angle) + (1 - alpha) * acc_Angle;
  pre_angle = current_angle;
  return current_angle;
}


// Low Pass Filter
LowPass::LowPass(float a){
  alpha = a;
  for(int i=0; i<2; i++){
    for(int j = 0;j < 2; j++) c[i][j] = 0;
  }
}

float LowPass::update_(float value){
  c[1][0] = value;
  c[0][0] = alpha * c[0][1] + ((1 - alpha) / 2) * (c[1][0] + c[1][1]);
  c[0][1] = c[0][0];
  c[1][1] = c[1][0];
  return c[0][0];
}
