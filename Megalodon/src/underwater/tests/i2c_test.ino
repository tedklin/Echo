#include <Wire.h>

int address_sensor1= 0x28;
int address_sensor2= 0x29;

void setup() {
  Serial.begin(9600); //this creates the Serial Monitor
  Wire.begin(); //this creates a Wire object
}

void loop() {
  Wire.beginTransmission(address_sensor1); //Send a request to begin communication with the device at the specified address

  Wire.write(0); //Sends a bit asking for register 0, the data register of the TC74 sensor
  
  Wire.endTransmission(); //this ends transmission of data from the arduino to the temperature sensor
  
  //this now reads the temperature from the TC74 sensor
  Wire.requestFrom(address_sensor1, 1);//this requests 1 byte from the specified address
  
  while(Wire.available() == 0);
  int celsius1= Wire.read();
  
  int fahrenheit1= round(celsius1 * 9.0/5.0 + 32.0);
  
  Serial.print("Temperature sensor 1:");
  Serial.print(celsius1);
  Serial.print("degrees celsius ");
  Serial.print(fahrenheit1);
  Serial.print(" degrees Fahrenheit");
  
  delay(2000);
  Wire.beginTransmission(address_sensor2); //Send a request to begin communication with the device at the specified address
  
  Wire.write(0); //Sends a bit asking for register 0, the data register of the TC74 sensor
  
  Wire.endTransmission(); //this ends transmission of data from the arduino to the temperature sensor
  
  //this now reads the temperature from the TC74 sensor
  Wire.requestFrom(address_sensor2, 1);//this requests 1 byte from the specified address
  
  while(Wire.available() == 0);
  int celsius2= Wire.read();
  
  int fahrenheit2= round(celsius2 * 9.0/5.0 + 32.0);
  
  Serial.print("Temperature sensor 2:"); Serial.print(celsius2);
  Serial.print("degrees celsius ");
  Serial.print(fahrenheit2);
  Serial.print(" degrees Fahrenheit");
  
  delay(2000);
}
