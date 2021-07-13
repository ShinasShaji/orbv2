/*
Code to calibrate servos using a ds4 controller
*/

#include <Servo.h>


// Serial recieve variables
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;


// Servo variables
// Initially set to mid position
// Expands array as required
#define SERVOS 3

Servo joints[SERVOS];
int servoPins[SERVOS] = {3, 5, 6};
int servoStates[SERVOS] = {90, 90, 90};

// Max swing rate in degrees per second
int maxSwingRate = 5;
// Refresh time in milliseconds
unsigned int servoRefresh = 100;

unsigned long prevServo = millis();


// Controller state
// Number of independent state variables
#define STATES 6
#define MIDSTATE 10

// L3x2, R3x2, L2, R2
int controller[STATES] = {10, 10, 10, 10, 0, 0};

unsigned long prevController = millis();


// General states
unsigned long currentTime = millis();



void setup(){
  // Starting serial
  Serial.begin(115200);
  Serial.println("<ping>");
  
  // Setting up servos
  for (int i = 0; i < SERVOS; i ++){
    
    // Attach pins to the corresponding servo
    joints[i].attach(servoPins[i]);
  
    // Delay before continuing
    delay(500);
    
    currentTime = millis();
  }
}


void loop(){
  // Update loop time
  currentTime = millis();
  
  // Recieve data from serial
  recvMarked();

  if (newData){
    newData = false;
    extractControllerState();
  }  
  
  if ((currentTime-prevServo)>=servoRefresh){
    updateServoStates();
    
    writeStatesToServos();
  }
}


// Function to remove start and stop characters from serial message
void recvMarked(){
  static boolean recvInProgress = false;
  static byte ndx = 0;
  static char startMarker = '<';
  static char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // Terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}


/* 
Function to extract controller values from serial message based
on message start character
*/
void extractControllerState(){
  char * token = strtok(receivedChars, " ");
  
  if (*token == 'd'){
    token = strtok(NULL, " ");
    for (int i = 0; i < STATES; i ++){
      controller[i] = atoi(token);
      
      token = strtok(NULL, " ");
    }
  }
}


// Function to update servo states
void updateServoStates(){
  servoStates[0] = servoStates[0] + (maxSwingRate*(controller[0]-MIDSTATE)/MIDSTATE);
  servoStates[1] = servoStates[1] + (maxSwingRate*(controller[2]-MIDSTATE)/MIDSTATE);
  servoStates[2] = servoStates[2] + (maxSwingRate*(controller[5]-controller[4])/MIDSTATE);
  
  prevServo = currentTime;
}


// Function to write servoState to all servos
void writeStatesToServos(){
  for (int k = 0; k < SERVOS; k ++){
    joints[k].write(servoStates[k]);
  }
}
