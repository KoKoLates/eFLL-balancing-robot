#include <Fuzzy.h>
#include <Wire.h>

# define RMotor_DIG 4
# define RMotor_PWM 5
# define LMotor_DIG 6
# define LMotor_PWM 7

const int MPU = 0x68;
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float accAngle, gyroAngle, Angle;
float AccError, GyroError;
float elapsedTime, currentTime, previousTime;
float pwm;

float yn=0, yn1=0, xn1=0;
float a1=0, b=0, b1=0;

Fuzzy *fuzzy = new Fuzzy();

void setup(){
  Serial.begin(9600);
  pinMode(RMotor_DIG, OUTPUT);
  pinMode(RMotor_PWM, OUTPUT);
  pinMode(LMotor_DIG, OUTPUT);
  pinMode(LMotor_PWM, OUTPUT);
  //
  Wire.begin();
  Wire.beginTransmission(MPU); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  fuzzy_initialize();
  calculate_error();
//  setTimer();
}

void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  
  AccX = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0; 
  AccY = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0;
  AccZ = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0;
 
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);
  GyroX = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0 - GyroError;

  accAngle = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccError;
  yn = 0.95*yn1 + 0.025*accAngle + 0.025*xn1;
  xn1 = accAngle;
  yn1 = yn;

  previousTime = currentTime; 
  currentTime = millis(); 
  elapsedTime = (currentTime - previousTime) / 1000;
  b = 0.95*b1 + 0.025*GyroX + 0.025*a1;
  b1 = b; a1 = GyroX;
  gyroAngle += GyroX * elapsedTime;
  Angle = 0.1 * gyroAngle + 0.9 * yn;
  fuzzy -> setInput(1, Angle);
  fuzzy -> setInput(2, b);
  fuzzy -> fuzzify();
  pwm = fuzzy -> defuzzify(1);

  setMotor(-pwm * 15);
}

//ISR(TIMER1_COMPA_vect){
//  fuzzy -> setInput(1, Angle);
//  fuzzy -> setInput(2, b);
//  fuzzy -> fuzzify();
//  pwm = fuzzy -> defuzzify(1);
//}

void calculate_error(){
  int c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0;
    AccY = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0;
    AccZ = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0;
    AccError += ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));

    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroError += (GyroX / 131.0);
    c++;
  }
  AccError = AccError / 200;
  GyroError = GyroError / 200;
}

void fuzzy_initialize(){
  FuzzySet *angle_N = new FuzzySet(-45.0, -45.0, -45.0, 45.0);
  FuzzySet *angle_P = new FuzzySet(-45.0, 45.0, 45.0, 45.0);
  
  FuzzySet *velocity_N = new FuzzySet(-50.0, -50.0, -50.0, 50.0);
  FuzzySet *velocity_P = new FuzzySet(-50, 50, 50, 50);
  
  FuzzySet *power_N = new FuzzySet(-255.0, -255.0, -255.0, 0);
  FuzzySet *power_Z = new FuzzySet(-255.0, 0, 0, 255.0);
  FuzzySet *power_P = new FuzzySet(0, 255, 255.0, 255.0);

  FuzzyInput *angle_input = new FuzzyInput(1);
  angle_input -> addFuzzySet(angle_N);
  angle_input -> addFuzzySet(angle_P);
  fuzzy -> addFuzzyInput(angle_input);
  
  FuzzyInput *velocity_input = new FuzzyInput(2);
  velocity_input -> addFuzzySet(velocity_N);
  velocity_input -> addFuzzySet(velocity_P);
  fuzzy -> addFuzzyInput(velocity_input);
  
  FuzzyOutput *power = new FuzzyOutput(1);
  power ->addFuzzySet(power_N);
  power ->addFuzzySet(power_Z);
  power ->addFuzzySet(power_P);
  fuzzy -> addFuzzyOutput(power);
 
  // Building Fuzzy Rule
  FuzzyRuleAntecedent *NN = new FuzzyRuleAntecedent();
  FuzzyRuleConsequent *P = new FuzzyRuleConsequent();
  NN -> joinWithAND(angle_N, velocity_N);
  P -> addOutput(power_P);
  FuzzyRule *rule1 = new FuzzyRule(1, NN, P);
  fuzzy -> addFuzzyRule(rule1);

  FuzzyRuleAntecedent *NP = new FuzzyRuleAntecedent();
  FuzzyRuleAntecedent *PN = new FuzzyRuleAntecedent();
  FuzzyRuleAntecedent *ZZ = new FuzzyRuleAntecedent();
  FuzzyRuleConsequent *Z = new FuzzyRuleConsequent();
  NP -> joinWithAND(angle_N, velocity_P);
  PN -> joinWithAND(angle_P, velocity_N);
  ZZ -> joinWithOR(NP, PN);
  Z -> addOutput(power_Z);
  FuzzyRule *rule2 = new FuzzyRule(2, ZZ, Z);
  fuzzy -> addFuzzyRule(rule2);

  FuzzyRuleAntecedent *PP = new FuzzyRuleAntecedent();
  FuzzyRuleConsequent *N = new FuzzyRuleConsequent();
  PP -> joinWithAND(angle_P, velocity_P);
  N -> addOutput(power_N);
  FuzzyRule *rule3 = new FuzzyRule(3, PP, N);
  fuzzy -> addFuzzyRule(rule3);
}

void setMotor(int MSpeed){
//  cli();
  MSpeed = constrain(MSpeed, -255, 255);
  Serial.println(MSpeed);
  if(MSpeed >= 0){
    analogWrite(LMotor_PWM, MSpeed);
    digitalWrite(LMotor_DIG, LOW);
    analogWrite(RMotor_PWM, MSpeed);
    digitalWrite(RMotor_DIG, LOW);
  }
  else{
    analogWrite(LMotor_PWM, MSpeed + 255);
    digitalWrite(LMotor_DIG, HIGH);
    analogWrite(RMotor_PWM, MSpeed + 255);
    digitalWrite(RMotor_DIG, HIGH);
  }
//  sei();
}

//void setTimer(){
//  cli();
//  TCCR1A = 0;
//  TCCR1B = 0;
//  OCR1A = 9999;
//  TCCR1B |= (1 << WGM12);
//  TCCR1B |= (1 << CS11);
//  TIMSK1 |= (1 << OCIE1A);
//  sei();
//}
