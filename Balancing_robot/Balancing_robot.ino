#include <Fuzzy.h>
#include <Wire.h>

# define RMotor_DIG 4
# define RMotor_PWM 5
# define LMotor_DIG 6
# define LMotor_PWM 7
# define MPU 0x68

float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float accAngle, gyroAngle, Angle;
float AccError, GyroError;
float elapsedTime, currentTime, previousTime;

float yn=0, yn1=0, xn1=0;
float a1=0, b=0, b1=0;

//Fuzzy *fuzzy = new Fuzzy();

void calculate_error(){
  int c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccError = AccError + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroError = GyroError + (GyroX / 131.0);
//    GyroErrorY = GyroErrorY + (GyroY / 131.0);
//    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  AccError = AccError / 200;
  GyroError = GyroError / 200;
}

//void fuzzy_initialize(){
//  FuzzySet *angle_N = new FuzzySet(-15, -15, -15, 15);
//  FuzzySet *angle_P = new FuzzySet(-15, 15, 15, 15);
//  
//  FuzzySet *velocity_N = new FuzzySet(-25, -25, -25, 25);
//  FuzzySet *velocity_P = new FuzzySet(-25, -25, -25, 25);
//  
//  FuzzySet *power_N = new FuzzySet(-255, -255, -255, 0);
//  FuzzySet *power_Z = new FuzzySet(-255, 0, 0, 255);
//  FuzzySet *power_P = new FuzzySet(0, 255, 255, 255);
//
//  FuzzyInput *angle = new FuzzyInput(1);
//  angle -> addFuzzySet(angle_N);
//  angle -> addFuzzySet(angle_P);
//  
//  FuzzyInput *velocity = new FuzzyInput(2);
//  velocity -> addFuzzySet(velocity_N);
//  velocity -> addFuzzySet(velocity_P);
//  
//  FuzzyOutput *power = new FuzzyOutput(1);
//  power ->addFuzzySet(power_N);
//  power ->addFuzzySet(power_Z);
//  power ->addFuzzySet(power_P);
//
//  // Building Fuzzy Rule
//  FuzzyRuleAntecedent *NN = new FuzzyRuleAntecedent();
//  FuzzyRuleConsequent *P = new FuzzyRuleConsequent();
//  NN -> joinWithAND(angle_N, velocity_N);
//  P -> addOutput(power_P);
//  FuzzyRule *rule1 = new FuzzyRule(1, NN, P);
//  fuzzy -> addFuzzyRule(rule1);
//
//  FuzzyRuleAntecedent *NP = new FuzzyRuleAntecedent();
//  FuzzyRuleAntecedent *PN = new FuzzyRuleAntecedent();
//  FuzzyRuleAntecedent *ZZ = new FuzzyRuleAntecedent();
//  FuzzyRuleConsequent *Z = new FuzzyRuleConsequent();
//  NP -> joinWithAND(angle_N, velocity_P);
//  PN -> joinWithAND(angle_P, velocity_N);
//  ZZ -> joinWithOR(NP, PN);
//  Z -> addOutput(power_Z);
//  FuzzyRule *rule2 = new FuzzyRule(2, ZZ, Z);
//  fuzzy -> addFuzzyRule(rule2);
//
//  FuzzyRuleAntecedent *PP = new FuzzyRuleAntecedent();
//  FuzzyRuleConsequent *N = new FuzzyRuleConsequent();
//  PP -> joinWithAND(angle_P, velocity_P);
//  N -> addOutput(power_N);
//  FuzzyRule *rule3 = new FuzzyRule(3, PP, N);
//  fuzzy -> addFuzzyRule(rule3);
//}

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
//  fuzzy_initialize();
  calculate_error();
  delay(20);
}

void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  
  AccX = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0; 
  AccY = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0;
  AccZ = (Wire.read() << 8 | Wire.read()) * 2 / 32768.0;
  accAngle = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccError;
  yn = 0.95*yn1 + 0.025*accAngle + 0.025*xn1;
  xn1 = accAngle;
  yn1 = yn;

  previousTime = currentTime; 
  currentTime = millis(); 
  elapsedTime = (currentTime - previousTime) / 1000; 
  Wire.beginTransmission(MPU);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = ((Wire.read() << 8 | Wire.read()) * 250 / 32768.0) - GyroError;
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();
  b = 0.95*b1 + 0.025*GyroX + 0.025*a1;
  b1 = b; a1 = GyroX;
  Serial.print(b);
  gyroAngle += GyroX * elapsedTime;
  Angle = 0.1 * gyroAngle + 0.9 * yn;
  Serial.print(" ,");
  Serial.println(Angle);

//  fuzzy -> setInput(1, Angle);
//  fuzzy -> setInput(2, b);
//  fuzzy -> fuzzify();
//
//  float pwm = fuzzy -> defuzzify(1);
//  Serial.print(", ");
//  Serial.println(pwm);
//
//  delay(100);
}
