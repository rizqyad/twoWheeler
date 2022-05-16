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
  err.pwr = map(err.pwr,0, 255, 30, 250);
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
