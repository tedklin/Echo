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
  imu::Vector<3> euler = m_imu.getVector(Adafruit_BNO055::VECTOR_EULER);
  m_measuredYaw = euler.x();
  m_measuredRoll = euler.y();
  m_measuredPitch = euler.z();


  // limited to +/- 360
  m_measuredYaw = 360 - m_measuredYaw;
  m_measuredYaw = fmod(m_measuredYaw, 360);
  m_measuredPitch = 360 - m_measuredPitch;
  m_measuredPitch = fmod(m_measuredPitch, 360);
  m_measuredRoll = 360 - m_measuredRoll;
  m_measuredRoll = fmod(m_measuredRoll, 360);
}

float m_yawError, m_rollError, m_pitchError;

float stabilize() {
  m_yawError = m_desiredYaw - m_measuredYaw;
  m_rollError = m_desiredRoll - m_measuredRoll;
  m_pitchError = m_desiredPitch - m_measuredPitch;

  m_yawError = (m_yawError > 180) ? m_yawError - 360 : m_yawError;
  m_yawError = (m_yawError < -180) ? m_yawError + 360 : m_yawError;
  m_pitchError = (m_pitchError > 180) ? m_pitchError - 360 : m_pitchError;
  m_pitchError = (m_pitchError < 180) ? m_pitchError + 360 : m_pitchError;
  m_rollError = (m_rollError > 180) ? m_rollError - 360 : m_rollError;
  m_rollError = (m_rollError < 180) ? m_rollError - 360 : m_rollError;
  
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

void displayStates() {
  Serial.print("Measured Yaw: ");
  Serial.println(m_measuredYaw);
  Serial.print("Measured Roll: ");
  Serial.println(m_measuredRoll);
  Serial.print("Measured Pitch: ");
  Serial.println(m_measuredPitch);

  Serial.print("Desired Yaw: ");
  Serial.println(m_desiredYaw);
  Serial.print("Desired Roll: ");
  Serial.println(m_desiredRoll);
  Serial.print("Desired Pitch: ");
  Serial.println(m_desiredPitch);
}

void displayInputs() {
  Serial.print("RC Receiver Channel 1: ");
  Serial.println(RC.getChannel(1));
  Serial.print("RC Receiver Channel 2: ");
  Serial.println(RC.getChannel(2));
  Serial.print("RC Receiver Channel 3: ");
  Serial.println(RC.getChannel(3));
}

void displayOutputs() {
  Serial.print("Horizontal Left Throttle: " );
  Serial.println(horizontalLeftThrottle);
  Serial.print("Horizontal Right Throttle: " );
  Serial.println(horizontalRightThrottle);
  
  Serial.print("Vertical Left Front Throttle: " );
  Serial.println(verticalLeftFrontThrottle);
  Serial.print("Vertical Left Back Throttle: " );
  Serial.println(verticalLeftBackThrottle);
  Serial.print("Vertical Right Front Throttle: " );
  Serial.println(verticalRightFrontThrottle);
  Serial.print("Vertical Right Back Throttle: " );
  Serial.println(verticalRightBackThrottle);
}

void setup() {
  Serial.begin(115200);
  initializeMotors();
  instantiateIMU();
  
  RC.begin();

  RC.addChannel(1); // forwards/reverse
  RC.addChannel(2); // turning
  RC.addChannel(3); // up/down

  RC.setMap(channel1_min, channel1_max, 1);
  RC.setMap(channel2_min, channel2_max, 2);
  RC.setMap(channel3_min, channel3_max, 3);
}

void loop() {
  updateStateEstimation();
  calculateThrottles();
  runMotors();
  
  displayStates();
  displayInputs();
  displayOutputs();

  //Wait till the refreshrate is expiered
  unsigned long waitSince = millis();
  while ((millis() - waitSince) < REFRESHRATE);
}
