#include <Servo.h>

byte servoPin = 9;
float throttle = 0;
Servo motor1;

// set power from -1.0 to 1.0
void setThrottle(float motorThrottle) {
  float input = motorThrottle * 400 + 1500;
  motor1.writeMicroseconds(input);
  Serial.print("Motor 1 Throttle: ");
  Serial.print(motorThrottle);
  Serial.print(" || Motor 1 input: ");
  Serial.print(input);
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  
  motor1.attach(servoPin);
  setThrottle(0);
  delay(7000);
}

void loop() {
  setThrottle(0.3);

  
}

