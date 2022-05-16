//IMU sensor
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <HMC5883L_Simple.h>
#include <Kalman.h>  // Source: https://github.com/TKJElectronics/KalmanFilter

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

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead

MPU6050 accelgyro;
HMC5883L_Simple Compass;
Kalman kalmanY;

// accelero and gyro
int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t tempRaw;
uint32_t timer;

double kalAngleY; // Calculated angle using a Kalman filter

// motor var
int16_t motor_speed;
// set point
float sp_bottom = -1.0;
float sp_top = 1.0;

// motor position
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
  // connection with IMU:
  Wire.begin();
  setupMPU();
  setupHMC();

  delay(500);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Accelerometers sensitivity:
  // -/+2g = 16384  LSB/g
  float xGf = (float)ax / (float)16384;
  float yGf = (float)ay / (float)16384;
  float zGf = (float)az / (float)16384;
  double pitch = calc_pitch(xGf, yGf, zGf);

  kalmanY.setAngle(pitch); // Set starting angle
  
  timer = micros();
  
  // Motor connection:
  pinMode(ENC1A, INPUT);
  pinMode(ENC1B, INPUT);
  pinMode(ENC2A, INPUT);
  pinMode(ENC2B, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //Serial.println("target pos");
}

void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // display tab-separated accel/gyro x/y/z values
//  Serial.print("a/g:\t");
//  Serial.print(ax); Serial.print("\t");
//  Serial.print(ay); Serial.print("\t");
//  Serial.print(az); Serial.print("\t");
//  Serial.print(gx); Serial.print("\t");
//  Serial.print(gy); Serial.print("\t");
//  Serial.println(gz);
//  // get heading
  float heading = Compass.GetHeadingDegrees();
//  Serial.print("Heading: \t");
//  Serial.println( heading );

  // Accelerometers sensitivity:
  // -/+2g = 16384  LSB/g
  float xGf = (float)ax / (float)16384;
  float yGf = (float)ay / (float)16384;
  float zGf = (float)az / (float)16384;
  //Convert Gyro rate 
  double gyroXrate = gx / 131.0; // Convert to deg/s
  double gyroYrate = gy / 131.0; // Convert to deg/s


  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  
  // Calculate Pitch from accelerometer (deg)
  double pitch = calc_pitch(xGf, yGf, zGf);
  
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Set angle
  Serial.println(kalAngleY);
  
  motor_speed = 95;
  if(pitch > sp_top){
    setMotor(1, motor_speed, PWM1, IN1, IN2);
    setMotor(1, motor_speed, PWM2, IN3, IN4);
  }else if(pitch < sp_bottom){
    setMotor(-1, motor_speed, PWM1, IN1, IN2);
    setMotor(-1, motor_speed, PWM2, IN3, IN4);
  }else{
    setMotor(-1, 0, PWM1, IN1, IN2);
    setMotor(-1, 0, PWM2, IN3, IN4);
  }
  
  // signal the motor
//  setMotor(err1.dir, err1.pwr, PWM1, IN1, IN2);
//  setMotor(err2.dir, err2.pwr, PWM2, IN3, IN4);

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
