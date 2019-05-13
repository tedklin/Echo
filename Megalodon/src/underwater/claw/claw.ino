#include "Servo.h"

Servo servo1;

#define INPUT_SIZE 30

bool open = false;
int pos1 = 0;
int pos2 = 0;

void receiveSerial() {
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

      if (strcmp(commandType, "claw1") == 0) {
        pos1 = input;
      } else if (strcmp(commandType, "claw2") == 0) {
        pos2 = input;
      }
    }
    // Find the next command in input string
    command = strtok(0, "&");
  }
}


void setup()
{
  servo1.attach(9);
  Serial.begin(9600);
}

void loop() {
//  int time = millis() % 10000;
////  Serial.println(time);
//  
//  if (time > 5000) {
//    servo1.write(180);
////    Serial.println("opening");
//  } else {
//    servo1.write(0);
////    Serial.println("closing");
//  }

  receiveSerial();
  Serial.println(servo1.read());
  servo1.write(pos1); // 100 and 65
  
//  if (open) {
//    servo1.write(180);
//    Serial.println("opening");
//  } else {
//    servo1.write(0);
//    Serial.println("closing");
//  }
}
