// Calculate based on max input size expected for one command
#define INPUT_SIZE 30

float inputArray[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// {hL = 0, hR = 1, vFL = 2, vFR = 3, vBL = 4, vBR = 5} ///

// 0.2&0.2&0.2&0.2&0.2&0.2
// hL:0.2&hR:0.2&vFL:0.2&vFR:0.2&vBL:0.2&vBR:0.2

//const char* hL = "hL";
//const char* hR = "hR";
//const char* vFL = "vFL";
//const char* vFR = "vFR";
//const char* vBL = "vBL";
//const char* vBR = "vBR";

void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
  // Get next command from Serial (add 1 for final 0)
  char input[INPUT_SIZE + 1];
  byte size = Serial.readBytes(input, INPUT_SIZE);
  // Add the final 0 to end the C string
  input[size] = 0;
  Serial.println("afid");

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

      Serial.println(command);
      Serial.println("Storing stuff");

      if (strcmp(commandType, "hL") == 0) {
        inputArray[0] = input;
        Serial.println("Storing 0");
      } else if (strcmp(commandType, "hR") == 0) {
        inputArray[1] = input;
        Serial.println("Storing 1");
      } else if (strcmp(commandType, "vFL") == 0) {
        inputArray[2] = input;
        Serial.println("Storing 2");
      } else if (strcmp(commandType, "vFR") == 0) {
        inputArray[3] = input;
        Serial.println("Storing 3");
      } else if (strcmp(commandType, "vBL") == 0) {
        inputArray[4] = input;
        Serial.println("Storing 4");
      } else if (strcmp(commandType, "vBR") == 0) {
        inputArray[5] = input;
        Serial.println("Storing 5");
      }
    }
    // Find the next command in input string
    command = strtok(0, "&");
    Serial.print("Calculating: ");
    Serial.println(index);
  }

  // Print current command
  index = 0;
  while (index < 6) {
    Serial.print("Index ");
    Serial.print(index);
    Serial.print(": ");
    Serial.println(inputArray[index]);
    index++;
  }
  Serial.println();
}
