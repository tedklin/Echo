#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <MS5837.h>
#include <utility/imumaths.h>

// ======================================================================================= //
//                                                                      START OF CONSTANTS // 
// ======================================================================================= //

#define LOOP_TIME_DELAY_MS (100)

const float kYawP = 0;
const float kYawI = 0;
const float kYawD = 0;
const float kPitchP = 0;
const float kPitchI = 0;
const float kPitchD = 0;
const float kRollP = 0;
const float kRollI = 0;
const float kRollD = 0;
const float kDepthP = 0;
const float kDepthI = 0;
const float kDepthD = 0;

const float kTranslationP = 0;
const float kTranslationI = 0;
const float kTranslationD = 0;

const float kYawThreshold = 5;
const float kPitchThreshold = 5;
const float kRollThreshold = 5;
const float kDepthThreshold = 5;

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
MS5837 m_barometer;

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
  while(!m_imu.begin())
  {
    Serial.println("IMU INIT FAILED");
    delay(5000);
  }
}

void instantiateBarometer() {
  Wire.begin();
  while (!m_barometer.init()) {
    Serial.println("BAROMETER INIT FAILED!");
    delay(5000);
  }
  m_barometer.setModel(MS5837::MS5837_30BA);
  m_barometer.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
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

float m_measuredDepth = 0;
float m_measuredAltitude = 0;
float m_measuredPressure = 0;

void updateIMU() {
  imu::Vector<3> euler = m_imu.getVector(Adafruit_BNO055::VECTOR_EULER);
  m_measuredYaw = euler.x();
  m_measuredRoll = euler.y();
  m_measuredPitch = euler.z();

  imu::Vector<3> linearAccel = m_imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  m_measuredX = linearAccel.x();
  m_measuredY = linearAccel.y();
  m_measuredZ = linearAccel.z();
}

void updateBarometer() {
  m_barometer.read();
  m_measuredDepth = m_barometer.depth();
  m_measuredAltitude = m_barometer.altitude();
  m_measuredPressure = m_barometer.pressure();
}

void displaySensorStatus() {
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  uint8_t system, gyro, accel, mag = 0;
  m_imu.getCalibration(&system, &gyro, &accel, &mag);
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
float m_depthControlOutput = 0;
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
 * Update control loop output
 * @param desiredYaw
 * @param desiredRoll
 * @param desiredPitch
 */
void rotate(float desiredYaw, float desiredRoll, float desiredPitch) {
  m_pitchControlOutput = kPitchP * (desiredPitch - m_measuredPitch);
  m_rollControlOutput = kRollP * (desiredRoll - m_measuredRoll);
  m_yawControlOutput = kYawP * (desiredYaw - m_measuredYaw);
  if (!isPitchAligned(desiredPitch)) {
    m_rollControlOutput = 0;
    m_yawControlOutput = 0;
  } else if (!isRollAligned(desiredRoll)) {
    m_yawControlOutput = 0;
  }
}

/**
 * Go to depth
 * @param desiredDepth
 */
void goToDepth(float desiredDepth) {
  rotate(m_measuredYaw, 0, 0);
  if (isPitchAligned(0) && isRollAligned(0)) {
    m_depthControlOutput = kDepthP * (desiredDepth - m_measuredDepth);
  } else {
    m_depthControlOutput = 0;
  }
}

bool isYawAligned(float desiredYaw) {
  return abs(m_measuredYaw - desiredYaw) < kYawThreshold;
}

bool isPitchAligned(float desiredPitch) {
  return abs(m_measuredPitch - desiredPitch) < kPitchThreshold;
}

bool isRollAligned(float desiredRoll) {
  return abs(m_measuredRoll - desiredRoll) < kRollThreshold;
}

bool isDepthReached(float desiredDepth) {
  return abs(m_measuredDepth - desiredDepth) < kDepthThreshold;
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

  Serial.println("BAROMETER INSTANTIATING");
  instantiateBarometer();
  delay(5000);
  Serial.println("BAROMETER INSTANTIATED");
}

void loop() {
  updateIMU();
  updateBarometer();

  float desiredYaw = 0;
  float desiredRoll = 0;
  float desiredPitch = 0;
  float desiredDepth = 0;
  m_translationOutput = 1;
  
  if (isDepthReached(desiredDepth)) {
    goToDepth(desiredDepth);
    m_translationOutput = 0;
  } else if (!isYawAligned(desiredYaw) && !isRollAligned(desiredRoll) && !isPitchAligned(desiredPitch)) {
    rotate(desiredYaw, desiredRoll, desiredPitch);
    m_translationOutput = 0;
  }
  
  m_horizontalRightPower = m_yawControlOutput + m_translationOutput;
  m_horizontalRightPower = -m_yawControlOutput + m_translationOutput;
  m_verticalFrontRightPower = m_rollControlOutput + m_pitchControlOutput + m_depthControlOutput;
  m_verticalFrontLeftPower = m_rollControlOutput - m_pitchControlOutput + m_depthControlOutput;
  m_verticalBackRightPower = -m_rollControlOutput + m_pitchControlOutput + m_depthControlOutput;
  m_verticalBackLeftPower = -m_rollControlOutput - m_pitchControlOutput + m_depthControlOutput;  
  
  updateMotorInput();

  delay(LOOP_TIME_DELAY_MS);
}

// ======================================================================================= //
//                                                                       #blueteambestteam //
// ======================================================================================= //
