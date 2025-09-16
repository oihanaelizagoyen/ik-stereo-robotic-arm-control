#include <Servo.h>

// Constants
const int BAUD_RATE = 31250;
const int NUMBER_OF_SERVOS = 6;
const int pins[NUMBER_OF_SERVOS] = {7, 6, 5, 4, 3, 2}; 
const int minimumAngles[NUMBER_OF_SERVOS] = {0, 0, 0, 0, 0, 10};
const int maximumAngles[NUMBER_OF_SERVOS] = {180, 150, 180, 180, 180, 60};
const bool control_time_log_enabled = true;

// Variables
Servo servos[NUMBER_OF_SERVOS];
int angles[NUMBER_OF_SERVOS];
int state_id;


void setup() {
  Serial.begin(BAUD_RATE);

  for (int i = 0; i < NUMBER_OF_SERVOS; i++){
    servos[i].attach(pins[i]);
    angles[i] = maximumAngles[i]/2;
  }
  moveServos();

}

void loop() {
  if (Serial.available() > 0){
    String line = Serial.readStringUntil('\n');
    int numberAnglesExtracted = extractAngles(line);

    if(numberAnglesExtracted == NUMBER_OF_SERVOS){
      moveServos();

      if (control_time_log_enabled) {
        Serial.print(state_id);
        Serial.print(" ");
      }
      for (int i = 0; i < NUMBER_OF_SERVOS; i++){
        Serial.print(angles[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
}

int extractAngles(String line){
  char *token = strtok((char *)line.c_str(), ","); 
  int i=0;
  
  if (control_time_log_enabled && token != NULL) {
    state_id = atoi(token);
    token = strtok(NULL, ",");
  }

  while (token != NULL && i < NUMBER_OF_SERVOS){
    angles[i] = constrain(atoi(token), minimumAngles[i], maximumAngles[i]);
    token = strtok(NULL, ",");
    i++;
  }

  return i;
}

void moveServos(){
  for (int i = 0; i < NUMBER_OF_SERVOS; i++){
    servos[i].write(constrain(angles[i], minimumAngles[i], maximumAngles[i]));
  }
}
