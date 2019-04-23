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

const float kYawP = 0.01;
const float kYawI = 0;
const float kYawD = 0;
const float kPitchP = 0.01;
const float kPitchI = 0;
const float kPitchD = 0;
const float kRollP = 0.01;
const float kRollI = 0;
const float kRollD = 0;
const float kDepthP = 0;
const float kDepthI = 0;
const float kDepthD = 0;

const float kTranslationP = 0;
const float kTranslationI = 0;
const float kTranslationD = 0;

const float kYawThreshold = 3;
const float kPitchThreshold = 3;
const float kRollThreshold = 3;
const float kDepthThreshold = 3;

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
  m_horizontalRightMotor.attach(11);
  m_verticalFrontLeftMotor.attach(5);
  m_verticalFrontRightMotor.attach(3);
  m_verticalBackLeftMotor.attach(10);
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

float m_measuredDepth = 0;
float m_measuredAltitude = 0;
float m_measuredPressure = 0;

void updateIMU() {
  imu::Vector<3> euler = m_imu.getVector(Adafruit_BNO055::VECTOR_EULER);
  m_measuredYaw = euler.x();
  m_measuredRoll = euler.y();
  m_measuredPitch = euler.z();
}

void limitOrientationMeasurements() {
  m_measuredYaw = fmod((360.0 - m_measuredYaw), 360.0);
  m_measuredRoll = fmod((360.0 - m_measuredRoll), 360.0);
  m_measuredPitch = fmod((360.0 - m_measuredPitch), 360.0);
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

float m_desiredYaw = 0;
float m_desiredRoll = 0;
float m_desiredPitch = 0;
float m_desiredDepth = 0;

float m_yawControlOutput = 0;
float m_rollControlOutput = 0;
float m_pitchControlOutput = 0;
float m_depthControlOutput = 0;
float m_translationControlOutput = 0;

float m_yawError = 0;
float m_rollError = 0;
float m_pitchError = 0;
float m_depthError = 0;
float m_translationError = 0;

float m_yawFromVisionB = 0;
float m_pitchFromVisionB = 0;
float m_rollFromVisionB = 0;
float m_transXFromVisionB = 0;
float m_transYFromVisionB = 0;
float m_transZFromVisionB = 0;

float m_yawFromVisionF = 0;
float m_pitchFromVisionF = 0;
float m_rollFromVisionF = 0;
float m_transXFromVisionF = 0;
float m_transYFromVisionF = 0;
float m_transZFromVisionF = 0;

float m_yawFromVisionR = 0;
float m_pitchFromVisionR = 0;
float m_rollFromVisionR = 0;
float m_transXFromVisionR = 0;
float m_transYFromVisionR = 0;
float m_transZFromVisionR = 0;

#define INPUT_SIZE 30

float directInputArray[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// direct input //
// hL:0.2&hR:0.2&vFL:0.2&vFR:0.2&vBL:0.2&vBR:0.2
// hL:0&hR:0&vFL:0&vFR:0&vBL:0&vBR:0
// yaw:0&pitch:0&roll:0

// autonomous input //
// 

void receiveSerialInput() {
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
        directInputArray[0] = input;
      } else if (strcmp(commandType, "hR") == 0) {
        directInputArray[1] = input;
      } else if (strcmp(commandType, "vFL") == 0) {
        directInputArray[2] = input;
      } else if (strcmp(commandType, "vFR") == 0) {
        directInputArray[3] = input;
      } else if (strcmp(commandType, "vBL") == 0) {
        directInputArray[4] = input;
      } else if (strcmp(commandType, "vBR") == 0) {
        directInputArray[5] = input;
      } else if (strcmp(commandType, "yaw") == 0) {
        m_desiredYaw = input;
      } else if (strcmp(commandType, "pitch") == 0) {
        m_desiredPitch = input;
      } else if (strcmp(commandType, "roll") == 0) {
        m_desiredRoll = input;
      } else if (strcmp(commandType, "depth") == 0) {
        m_desiredDepth = input;
      } else if (strcmp(commandType, "trans") == 0) {
        m_translationError = input;
      } else if (strcmp(commandType, "visYawB") == 0) {
        m_yawFromVisionB = input;
      } else if (strcmp(commandType, "visRollB") == 0) {
        m_rollFromVisionB = input;
      } else if (strcmp(commandType, "visPitchB") == 0) {
        m_pitchFromVisionB = input;
      } else if (strcmp(commandType, "visTransXB") == 0) {
        m_transXFromVisionB = input;
      } else if (strcmp(commandType, "visTransYB") == 0) {
        m_transYFromVisionB = input;
      } else if (strcmp(commandType, "visTransZB") == 0) {
        m_transZFromVisionB = input;
      } else if (strcmp(commandType, "visYawF") == 0) {
        m_yawFromVisionF = input;
      } else if (strcmp(commandType, "visRollF") == 0) {
        m_rollFromVisionF = input;
      } else if (strcmp(commandType, "visPitchF") == 0) {
        m_pitchFromVisionF = input;
      } else if (strcmp(commandType, "visTransXF") == 0) {
        m_transXFromVisionF = input;
      } else if (strcmp(commandType, "visTransYF") == 0) {
        m_transYFromVisionF = input;
      } else if (strcmp(commandType, "visTransZF") == 0) {
        m_transZFromVisionF = input;
      } else if (strcmp(commandType, "visYawR") == 0) {
        m_yawFromVisionR = input;
      } else if (strcmp(commandType, "visRollR") == 0) {
        m_rollFromVisionR = input;
      } else if (strcmp(commandType, "visPitchR") == 0) {
        m_pitchFromVisionR = input;
      } else if (strcmp(commandType, "visTransXR") == 0) {
        m_transXFromVisionR = input;
      } else if (strcmp(commandType, "visTransYR") == 0) {
        m_transYFromVisionR = input;
      } else if (strcmp(commandType, "visTransZR") == 0) {
        m_transZFromVisionR = input;
      } 
    }
    // Find the next command in input string
    command = strtok(0, "&");
  }
}

/**
 * Update control loop output
 * @param desiredYaw
 * @param desiredRoll
 * @param desiredPitch
 */
void rotate(float desiredYaw, float desiredRoll, float desiredPitch) {
  m_pitchError = desiredPitch - m_measuredPitch;
  m_rollError = desiredRoll - m_measuredRoll;
  m_yawError = desiredYaw - m_measuredYaw;

  m_pitchError = (m_pitchError > 180) ? m_pitchError - 360 : m_pitchError;
  m_pitchError = (m_pitchError < -180) ? m_pitchError + 360 : m_pitchError;
  m_rollError = (m_rollError > 180) ? m_rollError - 360 : m_rollError;
  m_rollError = (m_rollError < -180) ? m_rollError + 360 : m_rollError;
  m_yawError = (m_yawError > 180) ? m_yawError - 360 : m_yawError;
  m_yawError = (m_yawError < -180) ? m_yawError + 360 : m_yawError;
  
  m_pitchControlOutput = kPitchP * m_pitchError;
  m_rollControlOutput = kRollP * m_rollError;
  m_yawControlOutput = kYawP * m_yawError;
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
  m_depthError = desiredDepth - m_measuredDepth;
  
  if (isPitchAligned(0) && isRollAligned(0) ) {
    m_depthControlOutput = kDepthP * m_depthError;
  } else {
    m_depthControlOutput = 0;
  }
}

/**
 * Translate to based on error input
 */
void translate() {
  if (isYawAligned(m_desiredYaw) && isPitchAligned(m_desiredPitch) && isRollAligned(m_desiredRoll) && isDepthReached(m_desiredDepth)) {
    m_translationControlOutput = kTranslationP * m_translationError;
  } else {
    m_translationControlOutput = 0;
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

/**
 * @param float throttle (-1.0 to 1.0)
 * @return float input (microseconds to write to ESCs)
 */
float throttleToMicroseconds(float throttle) {
  if (throttle > 0.5) {
    throttle = 0.5;
  } else if (throttle < -0.5) {
    throttle = -0.5;
  }
  float input = throttle * 400 + 1500;
  return input;
}

/**
 * Actuate motors
 */
void runMotors() {
  m_horizontalLeftMotor.writeMicroseconds(throttleToMicroseconds(m_horizontalLeftPower));
  m_horizontalRightMotor.writeMicroseconds(throttleToMicroseconds(m_horizontalRightPower));
  m_verticalFrontLeftMotor.writeMicroseconds(throttleToMicroseconds(m_verticalFrontLeftPower));
  m_verticalFrontRightMotor.writeMicroseconds(throttleToMicroseconds(m_verticalFrontRightPower));
  m_verticalBackLeftMotor.writeMicroseconds(throttleToMicroseconds(m_verticalBackLeftPower));
  m_verticalBackRightMotor.writeMicroseconds(throttleToMicroseconds(m_verticalBackRightPower));

// // direct direct input stuff
//  m_horizontalLeftMotor.writeMicroseconds(throttleToMicroseconds(directInputArray[0]));
//  m_horizontalRightMotor.writeMicroseconds(throttleToMicroseconds(directInputArray[1]));
//  m_verticalFrontLeftMotor.writeMicroseconds(throttleToMicroseconds(directInputArray[2]));
//  m_verticalFrontRightMotor.writeMicroseconds(throttleToMicroseconds(directInputArray[3]));
//  m_verticalBackLeftMotor.writeMicroseconds(throttleToMicroseconds(directInputArray[4]));
//  m_verticalBackRightMotor.writeMicroseconds(throttleToMicroseconds(directInputArray[5]));
}

/**
 * Direct motor control
 */
void directMotorControl() {
  m_horizontalLeftPower = directInputArray[0];
  m_horizontalRightPower = directInputArray[1];
  m_verticalFrontLeftPower = directInputArray[2];
  m_verticalFrontRightPower = directInputArray[3];
  m_verticalBackLeftPower = directInputArray[4];
  m_verticalBackRightPower = directInputArray[5];
}

/**
 * Autonomous motor control
 */
void autonomousControl() {
  m_horizontalLeftPower = -m_yawControlOutput + m_translationControlOutput;
  m_horizontalRightPower = m_yawControlOutput + m_translationControlOutput;
  m_verticalFrontLeftPower = m_rollControlOutput - m_pitchControlOutput + m_depthControlOutput;
  m_verticalFrontRightPower = m_rollControlOutput + m_pitchControlOutput + m_depthControlOutput;
  m_verticalBackLeftPower = -m_rollControlOutput - m_pitchControlOutput + m_depthControlOutput;
  m_verticalBackRightPower = -m_rollControlOutput + m_pitchControlOutput + m_depthControlOutput;
}

/**
 * Stop motors
 */
void stopAll() {
//  m_horizontalRightMotor.writeMicroseconds(throttleToMicroseconds(0));
//  m_horizontalLeftMotor.writeMicroseconds(throttleToMicroseconds(0));
//  m_verticalFrontRightMotor.writeMicroseconds(throttleToMicroseconds(0));
//  m_verticalFrontLeftMotor.writeMicroseconds(throttleToMicroseconds(0));
//  m_verticalBackRightMotor.writeMicroseconds(throttleToMicroseconds(0));
//  m_verticalBackLeftMotor.writeMicroseconds(throttleToMicroseconds(0));

//  m_horizontalLeftMotor.writeMicroseconds(1500);
//  m_horizontalRightMotor.writeMicroseconds(1500);
//  m_verticalFrontLeftMotor.writeMicroseconds(1500);
//  m_verticalFrontRightMotor.writeMicroseconds(1500);
//  m_verticalBackLeftMotor.writeMicroseconds(1500);
//  m_verticalBackRightMotor.writeMicroseconds(1500);

  m_horizontalLeftPower = 0;
  m_horizontalRightPower = 0;
  m_verticalFrontLeftPower = 0;
  m_verticalFrontRightPower = 0;
  m_verticalBackLeftPower = 0;
  m_verticalBackRightPower = 0;
}

// ======================================================================================= //
//                                                                 END OF MOVEMENT METHODS //
// ======================================================================================= //
//                                                                                         //
// ======================================================================================= //
//                                                                   START OF AUTO METHODS //
// ======================================================================================= //

double calculateInitialYaw() {
  return atan2(m_transXFromVisionF, m_transZFromVisionF);
}

double calculateInitialDepth() {
  return m_transYFromVisionF;
}

// ======================================================================================= //
//                                                                      START OF MAIN CODE //
// ======================================================================================= //

void reportStates() {
  Serial.println("-----------");
//  Serial.print("hL : ");
//  Serial.println(m_horizontalLeftPower);
//  Serial.print("hR : ");
//  Serial.println(m_horizontalRightPower);
//  Serial.print("vFL : ");
//  Serial.println(m_verticalFrontLeftPower);
//  Serial.print("vFR : ");
//  Serial.println(m_verticalFrontRightPower);
//  Serial.print("vBL : ");
//  Serial.println(m_verticalBackLeftPower);
//  Serial.print("vBR : ");
//  Serial.println(m_verticalBackRightPower);
//  Serial.println("");

  Serial.print("Yaw Measured: ");
  Serial.println(m_measuredYaw);
  Serial.print("Roll Measured: " );
  Serial.println(m_measuredRoll);
  Serial.print("Pitch Measured: " );
  Serial.println(m_measuredPitch);
  Serial.print("Depth Measured: " );
  Serial.println(m_measuredDepth);

  Serial.print("Yaw Desired: ");
  Serial.println(m_desiredYaw);
  Serial.print("Roll Desired: " );
  Serial.println(m_desiredRoll);
  Serial.print("Pitch Desired: " );
  Serial.println(m_desiredPitch);
  Serial.print("Depth Desired: " );
  Serial.println(m_desiredDepth);

  Serial.print("Yaw Error: ");
  Serial.println(m_yawError);
  Serial.print("Roll Error: " );
  Serial.println(m_rollError);
  Serial.print("Pitch Error: " );
  Serial.println(m_pitchError);
  Serial.print("Depth Error: " );
  Serial.println(m_depthError);
  
  Serial.print("Yaw Control Output: ");
  Serial.println(m_yawControlOutput);
  Serial.print("Roll Control Output: " );
  Serial.println(m_rollControlOutput);
  Serial.print("Pitch Control Output: " );
  Serial.println(m_pitchControlOutput);
  Serial.print("Depth Control Output: " );
  Serial.println(m_depthControlOutput);
  Serial.println("-----------");
}

void setup() {
  Serial.begin(9600);

  Serial.println("MOTORS INSTANTIATING");
  instantiateMotors();
  stopAll();
  delay(10000);
  Serial.println("MOTORS INSTANTIATED");

  Serial.println("IMU INSTANTIATING");
  instantiateIMU();
  delay(5000);
  Serial.println("IMU INSTANTIATED");
//
//  Serial.println("BAROMETER INSTANTIATING");
//  instantiateBarometer();
//  delay(5000);
//  Serial.println("BAROMETER INSTANTIATED");
}

void loop() {
  receiveSerialInput();
  
  updateIMU();
  limitOrientationMeasurements();
//  updateBarometer();
  
//  if (!isDepthReached(m_desiredDepth)) {
//    goToDepth(m_desiredDepth);
//  } else if (!isYawAligned(m_desiredYaw) && !isRollAligned(m_desiredRoll) && !isPitchAligned(m_desiredPitch)) {
//    rotate(m_desiredYaw, m_desiredRoll, m_desiredPitch);
//  }

  rotate(m_desiredYaw, m_desiredRoll, m_desiredPitch);

  reportStates();

//  directMotorControl(); // direct serial input to motors
  autonomousControl();  // autonomous update input to motors
  runMotors(); // actuate motors 

  delay(LOOP_TIME_DELAY_MS);
}

// ======================================================================================= //
//                                                                       #blueteambestteam //
// ======================================================================================= //
