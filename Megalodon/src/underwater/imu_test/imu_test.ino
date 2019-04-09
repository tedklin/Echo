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

float innerCurrentMillis;
float innerPreviousMillis;

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
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
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
  innerPreviousMillis = millis() / 1000.0;
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

bool isIntermediateStep = false;

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

//  /* Display the floating point data */
//  Serial.print("Roll: ");
//  Serial.print(euler.x());
//  Serial.print(" Pitch: ");
//  Serial.print(euler.y());
//  Serial.print(" Yaw: ");
//  Serial.print(euler.z());
//  Serial.print("\t\t");

  imu::Vector<3> linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float linearAccelX = linearAccel.x();
  float linearAccelY = linearAccel.y();
  float linearAccelZ = linearAccel.z();

  currentMillis = millis() / 1000.0;
  float timeDifference = currentMillis - previousMillis;
  previousMillis = currentMillis;
  Serial.print(timeDifference);
  Serial.print("\t\t");

//  if (linearAccelX < 0) {
//    x += -linearAccelX * linearAccelX * timeDifference * timeDifference;
//  } else {
//    x += linearAccelX * linearAccelX * timeDifference * timeDifference;
//  }
//  if (linearAccelY < 0) {
//    y += -linearAccelY * linearAccelY * timeDifference * timeDifference;
//  } else {
//    y += linearAccelY * linearAccelY * timeDifference * timeDifference;
//  }
//  if (linearAccelZ < 0) {
//    z += -linearAccelZ * linearAccelZ * timeDifference * timeDifference;
//  } else {
//    z += linearAccelZ * linearAccelZ * timeDifference * timeDifference;
//  }

  if (abs(linearAccelX) < 0.2) {
    linearAccelX = 0;
  }
  if (abs(linearAccelY) < 0.2) {
    linearAccelY = 0;
  }
  if (abs(linearAccelZ) < 0.2) {
    linearAccelZ = 0;
  }

  if (abs(linearAccelX) < 0.08) {
    vel_x = 0;
  }
  if (abs(linearAccelY) < 0.08) {
    vel_y = 0;
  }
  if (abs(linearAccelZ) < 0.08) {
    vel_z = 0;
  }

  if (isIntermediateStep == false) {
    innerCurrentMillis = millis() / 1000.0;
    x += 0.5 * (vel_x + lastVelX) * (innerCurrentMillis - innerPreviousMillis);
    y += 0.5 * (vel_y + lastVelY) * (innerCurrentMillis - innerPreviousMillis);
    z += 0.5 * (vel_z + lastVelZ) * (innerCurrentMillis - innerPreviousMillis);
   
    lastVelX = vel_x;
    lastVelY = vel_y;
    lastVelZ = vel_z;

    innerPreviousMillis = innerCurrentMillis;
    isIntermediateStep = true;
  } else {
    isIntermediateStep = false;
  }

//  x += 0.5 * linearAccelX * timeDifference * timeDifference;
//  y += 0.5 * linearAccelY * timeDifference * timeDifference;
//  z += 0.5 * linearAccelZ * timeDifference * timeDifference;
  
  vel_x += 0.5 * (linearAccelX + lastLinearAccelX) * timeDifference;
  vel_y += 0.5 * (linearAccelY + lastLinearAccelY) * timeDifference;
  vel_z += 0.5 * (linearAccelZ + lastLinearAccelZ) * timeDifference;

  lastLinearAccelX = linearAccelX;
  lastLinearAccelY = linearAccelY;
  lastLinearAccelZ = linearAccelZ;
  
  Serial.print("Accel X: ");
  Serial.print(linearAccelX);
  Serial.print(" Accel Y: ");
  Serial.print(linearAccelY);
  Serial.print(" Accel Z: ");
  Serial.print(linearAccelZ);
  Serial.print("\t");
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(x * 100);
  Serial.print(" Y: ");
  Serial.print(y * 100);
  Serial.print(" Z: ");
  Serial.println(z * 100);
//  Serial.print("\t\t");

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
