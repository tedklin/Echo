#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ======================================================================================= //
//                                                                      START OF CONSTANTS // 
// ======================================================================================= //

#define kLoopTimeDelayMs (100);

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

// ======================================================================================= //
//                                                                        END OF CONSTANTS //
// ======================================================================================= //
//                                                                                         //
// ======================================================================================= //
//                                                         START OF HARDWARE INSTANTIATION //
// ======================================================================================= //

Servo m_horizontalRightMotor;
Servo m_horizontalLeftMotor;
Servo m_verticalFrontRightMotor;
Servo m_verticalFrontLeftMotor;
Servo m_verticalBackRightMotor;
Servo m_verticalBackLeftMotor;

Adafruit_BNO055 m_imu;

void instantiateMotors() {
  m_horizontalRightMotor.attach(5);
  m_horizontalLeftMotor.attach(6);
  m_verticalFrontRightMotor.attach(3);
  m_verticalFrontLeftMotor.attach(9);
  m_verticalBackRightMotor.attach(10);
  m_verticalBackLeftMotor.attach(11);
}

void instantiateIMU() {
  m_imu = Adafruit_BNO055();
  if(!m_imu.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
}

// ======================================================================================= //
//                                                           END OF HARDWARE INSTANTIATION //
// ======================================================================================= //
//                                                                                         //
// ======================================================================================= //
//                                                       START OF STATE ESTIMATION METHODS //
// ======================================================================================= //

float m_measuredYaw = 0;
float m_measuredPitch = 0;
float m_measuredRoll = 0;
float m_measuredX = 0;
float m_measuredY = 0;
float m_measuredZ = 0;

void updateIMU() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  m_measuredYaw = euler.x();
  m_measuredRoll = euler.y();
  m_measuredPitch = euler.z();

  imu::Vector<3> linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  m_measuredX = linearAccel.x();
  m_measuredY = linearAccel.y();
  m_measuredZ = linearAccel.z();
}

void displaySensorStatus() {
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
}

// ======================================================================================= //
//                                                         END OF STATE ESTIMATION METHODS //
// ======================================================================================= //
//                                                                                         //
// ======================================================================================= //
//                                                               START OF MOVEMENT METHODS //
// ======================================================================================= //

float m_horizontalRightPower = 0;
float m_horizontalLeftPower = 0;
float m_verticalFrontRightPower = 0;
float m_verticalFrontLeftPower = 0;
float m_verticalBackRightPower = 0;
float m_verticalBackLeftPower = 0;

float m_yawControlOutput = 0;
float m_rollControlOutput = 0;
float m_pitchControlOutput = 0;
float m_xControlOutput = 0;
float m_yControlOutput = 0;
float m_zControlOutput = 0;
float m_translationOutput = 0;

/**
 * @param Servo motor 
 * @param float throttle (-1.0 to 1.0)
 */
void setThrottle(Servo motor, float throttle) {
  float input = throttle * 400 + 1500;
  if (input > 1.0) {
    input = 1.0;
  } else if (input < -1.0) {
    input = -1.0;
  }
  motor.writeMicroseconds(input);
}

/**
 * Actuate motors
 */
void updateMotorInput() {
  setThrottle(m_horizontalRightMotor, m_horizontalRightPower);
  setThrottle(m_horizontalLeftMotor, m_horizontalLeftPower);
  setThrottle(m_verticalFrontRightMotor, m_verticalFrontRightPower);
  setThrottle(m_verticalFrontLeftMotor, m_verticalFrontLeftPower);
  setThrottle(m_verticalBackRightMotor, m_verticalBackRightPower);
  setThrottle(m_verticalBackLeftMotor, m_verticalBackLeftPower);
}

/**
 * Stop motors
 */
void stopAll() {
  setThrottle(m_horizontalRightMotor, 0.0);
  setThrottle(m_horizontalLeftMotor, 0.0);
  setThrottle(m_verticalFrontRightMotor, 0.0);
  setThrottle(m_verticalFrontLeftMotor, 0.0);
  setThrottle(m_verticalBackRightMotor, 0.0);
  setThrottle(m_verticalBackLeftMotor, 0.0);
}

/**
 * Rotation
 * @param kDesiredYaw
 * @param kDesiredPitch
 * @param kDesiredRoll
 */
void rotate(float desiredYaw, float desiredRoll, float desiredPitch) {
  m_yawControlOutput = kYawP * (desiredYaw - m_measuredYaw);
  m_rollControlOutput = kRollP * (desiredRoll - m_measuredRoll);
  m_pitchControlOutput = kPitchP * (desiredPitch - m_measuredPitch);
}

/**
 * Translation (depth control)
 * @param desiredZ
 */
void translate(float desiredZ) {
  m_zControlOutput = kZP * (desiredZ - m_measuredZ);
}

// ======================================================================================= //
//                                                                 END OF MOVEMENT METHODS //
// ======================================================================================= //
//                                                                                         //
// ======================================================================================= //
//                                                                      START OF MAIN CODE //
// ======================================================================================= //

void setup() {
  Serial.begin(9600);

  Serial.println("MOTORS INSTANTIATING");
  instantiateMotors();
  stopAll();
  delay(5000);
  Serial.println("MOTORS INSTANTIATED");

  Serial.println("IMU INSTANTIATING");
  instantiateIMU();
  delay(5000);
  Serial.println("IMU INSTANTIATED");
}

void loop() {
  rotate(0, 0, 0);
  translationOutput = 0;
  
  m_horizontalRightPower = m_yawControlOutput + translationOutput;
  m_horizontalRightPower = -m_yawControlOutput + translationOutput;
  m_verticalFrontRightPower = m_rollControlOutput + m_pitchControlOutput + m_zControlOutput;
  m_verticalFrontLeftPower = m_rollControlOutput - m_pitchControlOutput + m_zControlOutput;
  m_verticalBackRightPower = -m_rollControlOutput + m_pitchControlOutput + m_zControlOutput;
  m_verticalBackLeftPower = -m_rollControlOutput - m_pitchControlOutput + m_zControlOutput;  
  
  updateMotorInput();
  updateIMU();

  delay(kLoopTimeDelayMs);
}

// ======================================================================================= //
//                                                                       #blueteambestteam //
// ======================================================================================= //
