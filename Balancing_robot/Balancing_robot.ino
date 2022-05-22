#include <Fuzzy.h>

# define RMotor_DIG 4
# define RMotor_PWM 5
# define LMotor_DIG 6
# define LMotor_PWM 7

Fuzzy *fuzzy = new Fuzzy();

void fuzzy_set(){
  
}

void setup(){
  Serial.begin(9600);
  pinMode(RMotor_DIG, OUTPUT);
  pinMode(RMotor_PWM, OUTPUT);
  pinMode(LMotor_DIG, OUTPUT);
  pinMode(LMotor_PWM, OUTPUT);

  fuzzy_set();
}

void loop(){
  
}
