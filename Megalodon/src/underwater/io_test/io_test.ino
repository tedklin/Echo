// Calculate based on max input size expected for one command
#define INPUT_SIZE 30

float inputArray[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// {hL = 0, hR = 1, vFL = 2, vFR = 3, vBL = 4, vBR = 5} ///

// 0.2&0.2&0.2&0.2&0.2&0.2

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
    float input = atof(command);
    inputArray[index] = input;

//    // Split the command in two values
//    char* separator = strchr(command, ':');
//    if (separator != 0)
//    {
//      // Actually split the string in 2: replace ':' with 0
//      *separator = 0;
//      char* commandType = atoi(command);
//      ++separator;
//      int position = atoi(separator);
//
//      if (strcmp(commandType, "hL") == 0) {
//        
//      }
//      // Do something with servoId and position
//    }

    index++;
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
