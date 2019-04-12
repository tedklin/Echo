#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <MS5837.h>
#include <utility/imumaths.h>

// ======================================================================================= //
//                                                                      START OF CONSTANTS // 
// ======================================================================================= //

#define LOOP_TIME_DELAY_MS (200)

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

Servo m_horizontalLeftMotor;
Servo m_horizontalRightMotor;
Servo m_verticalFrontLeftMotor;
Servo m_verticalFrontRightMotor;
Servo m_verticalBackLeftMotor;
Servo m_verticalBackRightMotor;

Adafruit_BNO055 m_imu;
MS5837 m_barometer;

void instantiateMotors() {
  m_horizontalLeftMotor.attach(6);
  m_horizontalRightMotor.attach(5);
  m_verticalFrontLeftMotor.attach(10);
  m_verticalFrontRightMotor.attach(3);
  m_verticalBackLeftMotor.attach(11);
  m_verticalBackRightMotor.attach(9);
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

#define INPUT_SIZE 30

float inputArray[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// hL:0.2&hR:0.2&vFL:0.2&vFR:0.2&vBL:0.2&vBR:0.2
// hL:0&hR:0&vFL:0&vFR:0&vBL:0&vBR:0

void readFromSerial() {
  // Get next command from Serial (add 1 for final 0)
  char input[INPUT_SIZE + 1];
  byte size = Serial.readBytes(input, INPUT_SIZE);
  // Add the final 0 to end the C string
  input[size] = 0;

  // Read each command pair
  int index = 0;
  char* command = strtok(input, "&");
  while (command != 0) {
    // Split the command in two values
    char* separator = strchr(command, ':');
    if (separator != 0)
    {
      // Actually split the string in 2: replace ':' with 0
      *separator = 0;
      const char* commandType = command;
      ++separator;
      float input = atof(separator);

      if (strcmp(commandType, "hL") == 0) {
        inputArray[0] = input;
      } else if (strcmp(commandType, "hR") == 0) {
        inputArray[1] = input;
      } else if (strcmp(commandType, "vFL") == 0) {
        inputArray[2] = input;
      } else if (strcmp(commandType, "vFR") == 0) {
        inputArray[3] = input;
      } else if (strcmp(commandType, "vBL") == 0) {
        inputArray[4] = input;
      } else if (strcmp(commandType, "vBR") == 0) {
        inputArray[5] = input;
      }
    }
    // Find the next command in input string
    command = strtok(0, "&");
  }
}

/**
 * @param Servo motor 
 * @param float throttle (-1.0 to 1.0)
 */
float calculateThrottle(float throttle) {
  if (throttle > 1.0) {
    throttle = 1.0;
  } else if (throttle < -1.0) {
    throttle = -1.0;
  }
  float input = throttle * 400 + 1500;
  return input;
}

/**
 * Actuate motors
 */
void updateMotorInput() {
//  setThrottle(m_horizontalRightMotor, m_horizontalRightPower);
//  setThrottle(m_horizontalLeftMotor, m_horizontalLeftPower);
//  setThrottle(m_verticalFrontRightMotor, m_verticalFrontRightPower);
//  setThrottle(m_verticalFrontLeftMotor, m_verticalFrontLeftPower);
//  setThrottle(m_verticalBackRightMotor, m_verticalBackRightPower);
//  setThrottle(m_verticalBackLeftMotor, m_verticalBackLeftPower);

  m_horizontalLeftMotor.writeMicroseconds(calculateThrottle(inputArray[0]));
  m_horizontalRightMotor.writeMicroseconds(calculateThrottle(inputArray[1]));
  m_verticalFrontLeftMotor.writeMicroseconds(calculateThrottle(inputArray[2]));
  m_verticalFrontRightMotor.writeMicroseconds(calculateThrottle(inputArray[3]));
  m_verticalBackLeftMotor.writeMicroseconds(calculateThrottle(inputArray[4]));
  m_verticalBackRightMotor.writeMicroseconds(calculateThrottle(inputArray[5]));
}

/**
 * Stop motors
 */
void stopAll() {
//  m_horizontalRightMotor.writeMicroseconds(calculateThrottle(0));
//  m_horizontalLeftMotor.writeMicroseconds(calculateThrottle(0));
//  m_verticalFrontRightMotor.writeMicroseconds(calculateThrottle(0));
//  m_verticalFrontLeftMotor.writeMicroseconds(calculateThrottle(0));
//  m_verticalBackRightMotor.writeMicroseconds(calculateThrottle(0));
//  m_verticalBackLeftMotor.writeMicroseconds(calculateThrottle(0));

  m_horizontalLeftMotor.writeMicroseconds(1500);
  m_horizontalRightMotor.writeMicroseconds(1500);
  m_verticalFrontLeftMotor.writeMicroseconds(1500);
  m_verticalFrontRightMotor.writeMicroseconds(1500);
  m_verticalBackLeftMotor.writeMicroseconds(1500);
  m_verticalBackRightMotor.writeMicroseconds(1500);
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
  delay(10000);
  Serial.println("MOTORS INSTANTIATED");

//  Serial.println("IMU INSTANTIATING");
//  instantiateIMU();
//  delay(5000);
//  Serial.println("IMU INSTANTIATED");
//
//  Serial.println("BAROMETER INSTANTIATING");
//  instantiateBarometer();
//  delay(5000);
//  Serial.println("BAROMETER INSTANTIATED");
}

void loop() {
//  updateIMU();
//  updateBarometer();

  float desiredYaw = 0;
  float desiredRoll = 0;
  float desiredPitch = 0;
  float desiredDepth = 0;
  m_translationOutput = 0;
  
  if (isDepthReached(desiredDepth)) {
    goToDepth(desiredDepth);
    m_translationOutput = 0;
  } else if (!isYawAligned(desiredYaw) && !isRollAligned(desiredRoll) && !isPitchAligned(desiredPitch)) {
    rotate(desiredYaw, desiredRoll, desiredPitch);
    m_translationOutput = 0;
  }

  m_horizontalLeftPower = -m_yawControlOutput + m_translationOutput;
  m_horizontalRightPower = m_yawControlOutput + m_translationOutput;
  m_verticalFrontLeftPower = m_rollControlOutput - m_pitchControlOutput + m_depthControlOutput;
  m_verticalFrontRightPower = m_rollControlOutput + m_pitchControlOutput + m_depthControlOutput;
  m_verticalBackLeftPower = -m_rollControlOutput - m_pitchControlOutput + m_depthControlOutput;
  m_verticalBackRightPower = -m_rollControlOutput + m_pitchControlOutput + m_depthControlOutput;

  readFromSerial();
  updateMotorInput();

  delay(LOOP_TIME_DELAY_MS);
}

// ======================================================================================= //
//                                                                       #blueteambestteam //
// ======================================================================================= //
