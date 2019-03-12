#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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



// ======================================================================================= //
//                                                         END OF STATE ESTIMATION METHODS //
// ======================================================================================= //
//                                                                                         //
// ======================================================================================= //
//                                                               START OF MOVEMENT METHODS //
// ======================================================================================= //

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
 * Translation
 * @param kDesiredX
 * @param kDesiredY
 * @param kDesiredZ
 */
void translate(float desiredX, float desiredY, float desiredZ) {
  m_xControlOutput = kXP * (desiredX - m_measuredX);
  m_yControlOutput = kYP * (desiredY - m_measuredY);
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
  
  instantiateMotors();
  stopAll();
  delay(7000);
  Serial.println("MOTORS INSTANTIATED");

  
}

void loop() {
  rotate(0, 0, 0);
  translate(0, 0, 0);
  
  m_horizontalRightPower = m_yawControlOutput + m_xControlOutput;
  m_horizontalRightPower = -m_yawControlOutput + m_xControlOutput;
  m_verticalFrontRightPower = m_rollControlOutput + m_pitchControlOutput + m_zControlOutput;
  m_verticalFrontLeftPower = m_rollControlOutput - m_pitchControlOutput + m_zControlOutput;
  m_verticalBackRightPower = -m_rollControlOutput + m_pitchControlOutput + m_zControlOutput;
  m_verticalBackLeftPower = -m_rollControlOutput - m_pitchControlOutput + m_zControlOutput;  
  
  updateMotorInput();
}
