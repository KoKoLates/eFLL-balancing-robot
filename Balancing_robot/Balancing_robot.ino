# include <Fuzzy.h>
# include <filter.h>
# include <mpu6050.h>

# define RMotor_DIG 4
# define RMotor_PWM 5
# define LMotor_DIG 6
# define LMotor_PWM 7

Fuzzy *fuzzy = new Fuzzy();
MPU6050 MPU(0x68);
LowPass _acc(0.8);
LowPass _gyro(0.8);
Kalman kalman(0.001, 0.003, 0.03);

volatile float accAngle, gyroRate, Angle;
volatile float motor_power;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  pinMode(RMotor_DIG, OUTPUT);
  pinMode(RMotor_PWM, OUTPUT);
  pinMode(LMotor_DIG, OUTPUT);
  pinMode(LMotor_PWM, OUTPUT);

  // MPU6050 Initialize & Calibration
  MPU.MPU6050_INIT();
  MPU.MPU6050_CAL_();

  fuzzy_init();
  timer_init();  
}

void loop(){
  MPU.MPU6050_DATA();
  setMotor(motor_power * 10);
}

ISR(TIMER1_COMPA_vect){
  accAngle = MPU.ACC_ANGLE();
  gyroRate = MPU.GYRO_RATE();

  // Low Pass Filter
  accAngle = _acc.update_(accAngle);
  gyroRate = _gyro.update_(gyroRate);

  // Kalman Filter
  Angle = kalman.update_(accAngle, gyroRate);
  
  fuzzy -> setInput(1, Angle);
  fuzzy -> setInput(2, gyroRate);
  fuzzy -> fuzzify();
  motor_power = fuzzy -> defuzzify(1);
}

void fuzzy_init(){
  // Initialize Fuzzy Sets for inputs & output
  FuzzySet *angle_n = new FuzzySet(-20, -20, -5, 5);
  FuzzySet *angle_p = new FuzzySet(-5, 5, 20, 20);
  
  FuzzySet *velocity_n = new FuzzySet(-50, -50, -10, 10);
  FuzzySet *velocity_p = new FuzzySet(-10, 10, 50, 50);
  
  FuzzySet *power_n = new FuzzySet(-255, -255, -255, 0);
  FuzzySet *power_z = new FuzzySet(-255, 0, 0, 255);
  FuzzySet *power_p = new FuzzySet(0, 255, 255, 255);

  FuzzyInput *angle_input = new FuzzyInput(1);
  angle_input -> addFuzzySet(angle_n);
  angle_input -> addFuzzySet(angle_p);
  fuzzy -> addFuzzyInput(angle_input);
  
  FuzzyInput *velocity_input = new FuzzyInput(2);
  velocity_input -> addFuzzySet(velocity_n);
  velocity_input -> addFuzzySet(velocity_p);
  fuzzy -> addFuzzyInput(velocity_input);
  
  FuzzyOutput *power = new FuzzyOutput(1);
  power ->addFuzzySet(power_n);
  power ->addFuzzySet(power_z);
  power ->addFuzzySet(power_p);
  fuzzy -> addFuzzyOutput(power);
 
  // Building Fuzzy Rule
  FuzzyRuleAntecedent *nn = new FuzzyRuleAntecedent();
  FuzzyRuleConsequent *p = new FuzzyRuleConsequent();
  nn -> joinWithAND(angle_n, velocity_n);
  p -> addOutput(power_p);
  FuzzyRule *rule1 = new FuzzyRule(1, nn, p);
  fuzzy -> addFuzzyRule(rule1);

  FuzzyRuleAntecedent *np = new FuzzyRuleAntecedent();
  FuzzyRuleAntecedent *pn = new FuzzyRuleAntecedent();
  FuzzyRuleAntecedent *zz = new FuzzyRuleAntecedent();
  FuzzyRuleConsequent *z = new FuzzyRuleConsequent();
  np -> joinWithAND(angle_n, velocity_p);
  pn -> joinWithAND(angle_p, velocity_n);
  zz -> joinWithOR(np, pn);
  z -> addOutput(power_z);
  FuzzyRule *rule2 = new FuzzyRule(2, zz, z);
  fuzzy -> addFuzzyRule(rule2);

  FuzzyRuleAntecedent *pp = new FuzzyRuleAntecedent();
  FuzzyRuleConsequent *n = new FuzzyRuleConsequent();
  pp -> joinWithAND(angle_p, velocity_p);
  n -> addOutput(power_n);
  FuzzyRule *rule3 = new FuzzyRule(3, pp, n);
  fuzzy -> addFuzzyRule(rule3);
}

void timer_init(){
  // Timer1 initialize
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  // 1.6e7 /(prescaling * interupt frequncy) - 1
  // Sample time 0.008
  OCR1A = 15999;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void setMotor(int power){
  cli();
  power = constrain(power, -255, 255);
  if(power >= 0){
    analogWrite(LMotor_PWM, power);
    digitalWrite(LMotor_DIG, LOW);
    analogWrite(RMotor_PWM, power);
    digitalWrite(RMotor_DIG, LOW);
  }
  else{
    analogWrite(LMotor_PWM, power + 255);
    digitalWrite(LMotor_DIG, HIGH);
    analogWrite(RMotor_PWM, power + 255);
    digitalWrite(RMotor_DIG, HIGH);
  }
  sei();
}
