#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (20)

Adafruit_BNO055 bno = Adafruit_BNO055();

float vel_x;
float vel_y;
float vel_z;

float currentMillis;
float previousMillis;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(250000);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  vel_x = 0;
  vel_y = 0;
  vel_z = 0;

  previousMillis = millis() / 1000.0;
}

float lastLinearAccelX = 0;
float lastLinearAccelY = 0;
float lastLinearAccelZ = 0;

float lastVelX = 0;
float lastVelY = 0;
float lastVelZ = 0;

float x = 0;
float y = 0;
float z = 0;

float resetOffsetYaw = 0;
float resetOffsetPitch = 0;
float resetOffsetRoll = 0;


/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
// Possible vector values can be:
// - VECTOR_ACCELEROMETER - m/s^2
// - VECTOR_MAGNETOMETER  - uT
// - VECTOR_GYROSCOPE     - rad/s
// - VECTOR_EULER         - degrees
// - VECTOR_LINEARACCEL   - m/s^2
// - VECTOR_GRAVITY       - m/s^2

void loop(void)
{
  // --------- orientation ---------- //

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float yaw = euler.x() - resetOffsetYaw;
  float roll = euler.y() - resetOffsetPitch;
  float pitch = euler.z() - resetOffsetRoll;

  /* Display the floating point data */
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(" Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.println("\t\t");

  // --------- position ---------- //

  imu::Vector<3> linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float linearAccelX = linearAccel.x();
  float linearAccelY = linearAccel.y();
  float linearAccelZ = linearAccel.z();

  currentMillis = millis() / 1000.0;
  float timeDifference = currentMillis - previousMillis;
  previousMillis = currentMillis;
  Serial.print(timeDifference);
  Serial.print("\t\t");

  vel_x += (linearAccelX + lastLinearAccelX) / 2 * timeDifference;
  vel_y += (linearAccelY + lastLinearAccelY) / 2 * timeDifference;
  vel_z += (linearAccelZ + lastLinearAccelZ) / 2 * timeDifference;

  x += (vel_x + lastVelX) / 2 * timeDifference;
  y += (vel_y + lastVelY) / 2 * timeDifference;
  z += (vel_z + lastVelZ) / 2 * timeDifference;

  lastLinearAccelX = linearAccelX;
  lastLinearAccelY = linearAccelY;
  lastLinearAccelZ = linearAccelZ;

  lastVelX = vel_x;
  lastVelY = vel_y;
  lastVelZ = vel_z;

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.print(z);
  Serial.println("\t\t");

  //  // Quaternion data
  //  imu::Quaternion quat = bno.getQuat();
  //  Serial.print("qW: ");
  //  Serial.print(quat.w(), 4);
  //  Serial.print(" qX: ");
  //  Serial.print(quat.y(), 4);
  //  Serial.print(" qY: ");
  //  Serial.print(quat.x(), 4);
  //  Serial.print(" qZ: ");
  //  Serial.print(quat.z(), 4);
  //  Serial.print("\t\t");
  //
  //  /* Display calibration status for each sensor. */
  //  uint8_t system, gyro, accel, mag = 0;
  //  bno.getCalibration(&system, &gyro, &accel, &mag);
  //  Serial.print("CALIBRATION: Sys=");
  //  Serial.print(system, DEC);
  //  Serial.print(" Gyro=");
  //  Serial.print(gyro, DEC);
  //  Serial.print(" Accel=");
  //  Serial.print(accel, DEC);
  //  Serial.print(" Mag=");
  //  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
