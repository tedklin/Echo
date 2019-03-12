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

float kDesiredYaw = 0;
float kDesiredPitch = 0;
float kDesiredRoll = 0;
float kDesiredX = 0;
float kDesiredY = 0;
float kDesiredZ = 0;

float kHorizontalRightPower = 0;
float kHorizontalLeftPower = 0;
float kVerticalFrontRightPower = 0;
float kVerticalFrontLeftPower = 0;
float kVerticalBackRightPower = 0;
float kVerticalBackLeftPower = 0;

float kYawControlOutput = 0;
float kRollControlOutput = 0;
float kPitchControlOutput = 0;
float kXControlOutput = 0;
float kYControlOutput = 0;
float kZControlOutput = 0;

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

  updateMotorInput();
}

/**
 * @param Servo motor 
 * @param float throttle
 */
void setThrottle(Servo motor, float throttle) {
  float input = throttle * 400 + 1500;
  motor.writeMicroseconds(input);
  Serial.print("Motor 1 Throttle: ");
  Serial.print(throttle);
  Serial.print(" || Motor 1 input: ");
  Serial.print(input);
  Serial.println();
}

void updateMotorInput() {
  setThrottle(horizontalRightMotor, kHorizontalRightPower);
  setThrottle(horizontalLeftMotor, kHorizontalLeftPower);
  setThrottle(verticalFrontRightMotor, kVerticalFrontRightPower);
  setThrottle(verticalFrontLeftMotor, kVerticalFrontLeftPower);
  setThrottle(verticalBackRightMotor, kVerticalBackRightPower);
  setThrottle(verticalBackLeftMotor, kVerticalBackLeftPower);
}
