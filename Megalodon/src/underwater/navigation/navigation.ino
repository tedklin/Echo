#include <Servo.h>

Servo horizontalRightMotor;
Servo horizontalLeftMotor;
Servo verticalFrontRightMotor;
Servo verticalFrontLeftMotor;
Servo verticalBackRightMotor;
Servo verticalBackLeftMotor;

const float kYawP = 0;
const float kYawI = 0;
const float kYawD = 0;
const float kPitchP = 0;
const float kPitchI = 0;
const float kPitchD = 0;
const float kRollP = 0;
const float kRollI = 0;
const float kRollD = 0;

const float kXP = 0;
const float kXI = 0;
const float kXD = 0;
const float kYP = 0;
const float kYI = 0;
const float kYD = 0;
const float kZP = 0;
const float kZI = 0;
const float kZD = 0;

float horizontalRightPower = 0;
float horizontalLeftPower = 0;
float verticalFrontRightPower = 0;
float verticalFrontLeftPower = 0;
float verticalBackRightPower = 0;
float verticalBackLeftPower = 0;

float yawControlOutput = 0;
float rollControlOutput = 0;
float pitchControlOutput = 0;
float xControlOutput = 0;
float yControlOutput = 0;
float zControlOutput = 0;

void setup() {
  Serial.begin(9600);

  horizontalRightMotor.attach(5);
  horizontalLeftMotor.attach(6);
  verticalFrontRightMotor.attach(3);
  verticalFrontLeftMotor.attach(9);
  verticalBackRightMotor.attach(10);
  verticalBackLeftMotor.attach(11);

  stopAll();
  
  delay(7000);
}

void loop() {
  horizontalRightPower = yawControlOutput + xControlOutput;
  horizontalRightPower = -yawControlOutput + xControlOutput;
  verticalFrontRightPower = rollControlOutput + pitchControlOutput + zControlOutput;
  verticalFrontLeftPower = rollControlOutput - pitchControlOutput + zControlOutput;
  verticalBackRightPower = -rollControlOutput + pitchControlOutput + zControlOutput;
  verticalBackLeftPower = -rollControlOutput - pitchControlOutput + zControlOutput;  
  
  updateMotorInput();
}

/**
 * @param Servo motor 
 * @param float throttle (-1.0 to 1.0)
 */
void setThrottle(Servo motor, float throttle) {
  float input = throttle * 400 + 1500;
  motor.writeMicroseconds(input);
}

/**
 * Actuate motors
 */
void updateMotorInput() {
  setThrottle(horizontalRightMotor, horizontalRightPower);
  setThrottle(horizontalLeftMotor, horizontalLeftPower);
  setThrottle(verticalFrontRightMotor, verticalFrontRightPower);
  setThrottle(verticalFrontLeftMotor, verticalFrontLeftPower);
  setThrottle(verticalBackRightMotor, verticalBackRightPower);
  setThrottle(verticalBackLeftMotor, verticalBackLeftPower);
}

/**
 * Turn in place to orient in all 3 axis
 * @param kDesiredYaw
 * @param kDesiredPitch
 * @param kDesiredRoll
 */
void rotate(float desiredYaw, float desiredRoll, desiredPitch) {
  yawControlOutput = kYawP * (desiredYaw - measuredYaw);
  rollControlOutput = kRollP * (desiredRoll - measuredRoll);
  pitchControlOutput = kPitchP * (desiredPitch - measuredPitch);
}

/**
 * Translation movement
 * @param kDesiredX
 * @param kDesiredY
 * @param kDesiredZ
 */
void translate(float desiredX, float desiredY, float desiredZ) {
  xControlOutput = kXP * (desiredX - measuredX);
  yControlOutput = kYP * (desiredY - measuredY);
  zControlOutput = kZP * (desiredZ - measuredZ);
}
