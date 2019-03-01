#include <Servo.h>

Servo horizontalRightMotor;
Servo horizontalLeftMotor;
Servo verticalFrontRightMotor;
Servo verticalFrontLeftMotor;
Servo verticalBackRightMotor;
Servo verticalBackLeftMotor;

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

void stopAll() {
  setThrottle(horizontalRightMotor, 0.0);
  setThrottle(horizontalLeftMotor, 0.0);
  setThrottle(verticalFrontRightMotor, 0.0);
  setThrottle(verticalFrontLeftMotor, 0.0);
  setThrottle(verticalBackRightMotor, 0.0);
  setThrottle(verticalBackLeftMotor, 0.0);
}

// TODO: write function that takes pose and does internal PID
void 

