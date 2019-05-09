#include "FastRCReader.h"
#include <Servo.h>
#include <Adafruit_BNO055.h>

#define RECALIBRATE false

//Time definitions
#define REFRESHRATE 100

RCChannelMapper RC;

const uint16_t channel1_min = 1100;
const uint16_t channel1_max = 1900;
const uint16_t channel2_min = 1100;
const uint16_t channel2_max = 1900;
const uint16_t channel3_min = 1100;
const uint16_t channel3_max = 1900;

Servo horizontalLeftMotor;
Servo horizontalRightMotor;
Servo verticalLeftFrontMotor;
Servo verticalLeftBackMotor;
Servo verticalRightFrontMotor;
Servo verticalRightBackMotor;

float horizontalLeftThrottle = 0;
float horizontalRightThrottle = 0;
float verticalLeftFrontThrottle = 0;
float verticalLeftBackThrottle = 0;
float verticalRightFrontThrottle = 0;
float verticalRightBackThrottle = 0;

void initializeMotors() {
  horizontalLeftMotor.attach(1);
  horizontalRightMotor.attach(2);
  verticalLeftFrontMotor.attach(3);
  verticalLeftBackMotor.attach(4);
  verticalRightFrontMotor.attach(5);
  verticalRightBackMotor.attach(6);
}

Adafruit_BNO055 m_imu;

float m_measuredYaw = 0;
float m_measuredPitch = 0;
float m_measuredRoll = 0;

float m_desiredYaw = 0;
float m_desiredPitch = 0;
float m_desiredRoll = 0;

float m_yawControlOutput = 0;
float m_pitchControlOutput = 0;
float m_rollControlOutput = 0;

const float kYawP = 0.001;
const float kPitchP = 0.001;
const float kRollP = 0.001;

void instantiateIMU() {
  Serial.println("IMU INSTANTIATING");
  
  m_imu = Adafruit_BNO055();
  while(!m_imu.begin())
  {
    Serial.println("IMU INIT FAILED");
    delay(5000);
  }
  delay(5000);
  
  Serial.println("IMU INSTANTIATED");
}

float updateStateEstimation() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  m_measuredYaw = euler.x();
  m_measuredRoll = euler.y();
  m_measuredPitch = euler.z();
}

float stabilize() {
  m_yawError = m_desiredYaw - m_measuredYaw;
  m_rollError = m_desiredRoll - m_measuredRoll;
  m_pitchError = m_desiredPitch - m_measuredPitch;
  
  m_yawControlOutput = kYawP * m_yawError;
  m_rollControlOutput = kRollP * m_rollError;
  m_pitchControlOutput = kPitchP * m_pitchError;
}

float throttleToMicroseconds(float throttle) {
  if (throttle < -0.7) {
    throttle = -0.7;
  } else if (throttle > 0.7) {
    throttle = 0.7;
  }
  float input = throttle * 400 + 1500;
  return input;
}

void calculateThrottles() {
  horizontalLeftThrottle = RC.getChannel(1) + RC.getChannel(2);
  horizontalRightThrottle = RC.getChannel(1) - RC.getChannel(2);
  
  verticalLeftFrontThrottle = RC.getChannel(3);
  verticalLeftBackThrottle = RC.getChannel(3);
  verticalRightFrontThrottle = RC.getChannel(3);
  verticalRightBackThrottle = RC.getChannel(3);

  // stabilized
//  verticalLeftFrontThrottle = RC.getChannel(3) + m_rollControlOutput + m_pitchControlOutput;
//  verticalLeftBackThrottle = RC.getChannel(3) + m_rollControlOutput - m_pitchControlOutput;
//  verticalRightFrontThrottle = RC.getChannel(3) - m_rollControlOutput + m_pitchControlOutput;
//  verticalRightBackThrottle = RC.getChannel(3) - m_rollControlOutput - m_pitchControlOutput;
}

void runMotors() {
  horizontalLeftMotor.writeMicroseconds(throttleToMicroseconds(horizontalLeftThrottle));
  horizontalRightMotor.writeMicroseconds(throttleToMicroseconds(horizontalRightThrottle));
  verticalLeftFrontMotor.writeMicroseconds(throttleToMicroseconds(verticalLeftFrontThrottle));
  verticalLeftBackMotor.writeMicroseconds(throttleToMicroseconds(verticalLeftBackThrottle));
  verticalRightFrontMotor.writeMicroseconds(throttleToMicroseconds(verticalRightFrontThrottle));
  verticalRightBackMotor.writeMicroseconds(throttleToMicroseconds(verticalRightBackThrottle));
}

void setup() {
  //Initalise the Serial communication and the RC RX Reader
//  Serial.begin(115200);
  initializeMotors();
  initializeIMU();
  
  RC.begin();

  RC.addChannel(1); // forwards/reverse
  RC.addChannel(2); // turning
  RC.addChannel(3); // up/down

  RC.setMap(channel1_min, channel1_max, 1);
  RC.setMap(channel2_min, channel2_max, 2);
  RC.setMap(channel3_min, channel3_max, 3);
}


void loop() {
  calculateThrottles();
  runMotors();

  //Wait till the refreshrate is expiered
  unsigned long waitSince = millis();
  while ((millis() - waitSince) < REFRESHRATE);
}
