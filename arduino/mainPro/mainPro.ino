#include <util/atomic.h> // For the ATOMIC_BLOCK macro
// encoder 1 input
#define ENC1A 0 // YELLOW
#define ENC1B 2 // WHITE
// encoder 2 input
#define ENC2A 1 // YELLOW
#define ENC2B 3 // WHITE
// motor control
#define PWM1 10 // motor 1 speed control
// motor 1 rotation direction control
#define IN1 9
#define IN2 8
// motor 2 rotation direction control
#define IN3 7
#define IN4 6
#define PWM2 5 // motor 2 speed control

volatile int posi1 = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int posi2 = 0;
long prevT = 0;
struct typePIDM {
  float eprev = 0;
  float eintegral = 0;
  float pwr = 0;
  int dir = 1;
};
typePIDM err1;
typePIDM err2;

// Motor PID constants
float kp = 1;
float kd = 0.025;
float ki = 0.0;


void setup() {
  Serial.begin(9600);
  pinMode(ENC1A, INPUT);
  pinMode(ENC1B, INPUT);
  pinMode(ENC2A, INPUT);
  pinMode(ENC2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1A), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A), readEncoder2, RISING);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("target pos");
}

void loop() {

  // set target position
  //int target = 1200;
  int target = 90 * sin(prevT / 1e6);

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential misread if the interrupt coincides with this code running
  int pos1 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = posi1;
  }
  int pos2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos2 = posi2;
  }

  // motor control
  err1 = pidMotor(pos1, target, err1.eprev, err1.eintegral, deltaT, kp, kd, ki);
  err2 = pidMotor(pos2, target, err2.eprev, err2.eintegral, deltaT, kp, kd, ki);

  // signal the motor
  setMotor(err1.dir, err1.pwr, PWM1, IN1, IN2);
  setMotor(err2.dir, err2.pwr, PWM2, IN3, IN4);

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos1);
  Serial.print(" - ");
  Serial.print(pos2);
  Serial.println();
}

typePIDM pidMotor(int pos, int target, float eprev, float eintegral, float deltaT, float kp, float kd, float ki) {
  typePIDM err;
  err.eprev = eprev;
  err.eintegral = eintegral;
  // error
  int e = pos - target;
  // derivative
  float dedt = (e - err.eprev) / (deltaT);
  // integral
  err.eintegral = err.eintegral + e * deltaT;
  // control signal
  float u = kp * e + kd * dedt + ki * err.eintegral;

  // motor power
  err.pwr = fabs(u);
  if ( err.pwr > 255 ) {
    err.pwr = 255;
  }
  // motor direction
  err.dir = 1;
  if (u < 0) {
    err.dir = -1;
  }

  // store previous error
  err.eprev = e;

  return err;
}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder1() {
  int b = digitalRead(ENC1B);
  if (b > 0) {
    posi1++;
  }
  else {
    posi1--;
  }
}
void readEncoder2() {
  int b = digitalRead(ENC2B);
  if (b > 0) {
    posi2++;
  }
  else {
    posi2--;
  }
}
