/*
Code to calibrate servos using a ds4 controller
*/

#include <Servo.h>


// Serial recieve variables
#define BAUDRATE 115200
const byte numChars = 48;
char receivedChars[numChars];
boolean newData = false;
boolean serialConnected = false;
char testWord[] = "ping";
unsigned int pingInterval = 500;


// Servo variables
// Expands array as required
#define LEGS 4
#define SERVOS 12

Servo joints[SERVOS];
int servoPins[SERVOS] = {13, 12, 11,
                         10,  9,  8,
                          7,  6,  5,
                          4,  3,  2};

// Initially set to positions specified below
float servoStates[SERVOS] = {1500, 1500, 1500,    // Hip, shoulder, knee
                             1500, 1500, 1500,
                             1500, 1500, 1500,
                             1500, 1500, 1500};
                             
float servoAngles[SERVOS] = {0, 30, 0,    // Hip, shoulder, knee
                             0, 30, 0,
                             0, 30, 0,
                             0, 30, 0};

// Range of movement = {60, 90, 120} degrees for {hip, shoulder, knee}
  
int minServoStates[SERVOS] = {1125, 500, 800,
                              1000, 1000, 1000,
                              1175, 1825,  925,
                              1835,  915, 1725};
                              
int maxServoStates[SERVOS] = {1785, 2500, 1825,  
                              2000, 2000, 2000,
                              1825,  950, 1815,
                              1165, 1925, 780};
                              
int jointRanges[SERVOS] = {60, 90, 90,
                           60, 90, 90,
                           60, 90, 90,
                           60, 90, 90};

// Max swing rate in degrees per second
float maxSwingRate = 30;
// Refresh time in milliseconds
unsigned int servoRefresh = 10;

unsigned long prevServo = millis();


// Controller state
// Number of independent state variables
#define STATES 7
#define MIDSTATE 10

// L3x2, R3x2, L2, R2, Square
int controller[STATES] = {10, 10, 10, 10, 0, 0, 0};
int currentLeg = 0;
int legServoIndexOffset = 3*currentLeg;

unsigned long prevController = millis();


// General states
unsigned long currentTime = millis();



void setup(){
  // Establishing connection through serial
  establishSerialConnection();
  
  // Setting up servos
  maxSwingRate = maxSwingRate * servoRefresh / 1000;
  // initializeServoPosition();

  attachServoPins();
    
  currentTime = millis();
}


void loop(){
  // Update loop time
  currentTime = millis();
  
  // Recieve data from serial
  receiveSerialData();

  if (newData){
    newData = false;
    extractControllerState();
    checkLegChange();
    writeServoStateSerial();
  }  
  
  if ((currentTime-prevServo)>=servoRefresh){
    updateServoAngles();
    updateServoStates();
    
    writeStatesToServos();
  }
}


// Function to attach pins to corresponding servos
void attachServoPins() {
  // Updating servoAngles and servoStates to initialize
  updateServoAngles();
  updateServoStates();
  
  for (int i = 0; i < SERVOS; i ++){  
    // Attach pins to the corresponding servo
    joints[i].writeMicroseconds(servoStates[i]);
    joints[i].attach(servoPins[i]);
  
    // Delay before continuing
    delay(100);
  }
}


// Function to establish connection through serial
void establishSerialConnection() {
  Serial.begin(115200);
  
  while (!serialConnected) {
    
    receiveSerialData();
    
    if (newData) {
      newData = false;
      
      serialConnected = true;
      
      for (int i = 0; receivedChars[i]!='\0'; i++) {
        if (receivedChars[i]!=testWord[i]) {
          serialConnected = false;
          
          break;
        }
      }
      
      Serial.print("<");
      Serial.print(testWord);
      Serial.println(">");
    }
   
    else {
      delay(pingInterval);
      
      Serial.print("<");
      Serial.print(testWord);
      Serial.println(">");
    } 
  }  
}


// Function to remove start and stop characters from serial message
void receiveSerialData(){
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


// Function to update servo angles
void updateServoAngles() {
  legServoIndexOffset = 3*currentLeg;
  servoAngles[legServoIndexOffset+0] = servoAngles[legServoIndexOffset+0] + 
                                        (maxSwingRate*(controller[0] - MIDSTATE) / MIDSTATE);
   
  servoAngles[legServoIndexOffset+1] = servoAngles[legServoIndexOffset+1] + 
                                        (maxSwingRate*(controller[2] - MIDSTATE) / MIDSTATE);
                                        
  servoAngles[legServoIndexOffset+2] = servoAngles[legServoIndexOffset+2] + 
                                        (maxSwingRate*(controller[5] - controller[4]) / MIDSTATE);
                                        
  for (int k = 0; k < 3; k++) {
    if (servoAngles[legServoIndexOffset+k] < 0){
      servoAngles[legServoIndexOffset+k] = 0;
    }
    
    else if (servoAngles[legServoIndexOffset+k] > jointRanges[legServoIndexOffset+k]) {
      servoAngles[legServoIndexOffset+k] = jointRanges[legServoIndexOffset+k];
    }
  }
}


// Function to update servo states
void updateServoStates(){
  for (int legIndex = 0; legIndex < LEGS; legIndex++) {
    legServoIndexOffset = 3*legIndex;
  
    servoStates[legServoIndexOffset+0] = minServoStates[legServoIndexOffset+0] + (servoAngles[legServoIndexOffset+0] *
                                        (maxServoStates[legServoIndexOffset+0] - minServoStates[legServoIndexOffset+0]) / 
                                        jointRanges[legServoIndexOffset+0]);
                                        
    servoStates[legServoIndexOffset+1] = minServoStates[legServoIndexOffset+1] + (servoAngles[legServoIndexOffset+1] *
                                        (maxServoStates[legServoIndexOffset+1] - minServoStates[legServoIndexOffset+1]) / 
                                        jointRanges[legServoIndexOffset+1]);
   
    servoStates[legServoIndexOffset+2] = minServoStates[legServoIndexOffset+2] + (servoAngles[legServoIndexOffset+2] *
                                        (maxServoStates[legServoIndexOffset+2] - minServoStates[legServoIndexOffset+2]) / 
                                        jointRanges[legServoIndexOffset+2]);
  
    /*
    servoStates[legServoIndexOffset+0] = servoStates[legServoIndexOffset+0] + (maxSwingRate*(controller[0] - MIDSTATE) / MIDSTATE) * 
                                        ((maxServoStates[legServoIndexOffset+0] - minServoStates[legServoIndexOffset+0]) / 
                                        abs(maxServoStates[legServoIndexOffset+0] - minServoStates[legServoIndexOffset+0]));
                                        
    servoStates[legServoIndexOffset+1] = servoStates[legServoIndexOffset+1] + (maxSwingRate*(controller[2] - MIDSTATE) / MIDSTATE) * 
                                        (-(minServoStates[legServoIndexOffset+1] - maxServoStates[legServoIndexOffset+1]) / 
                                        abs(minServoStates[legServoIndexOffset+1] - maxServoStates[legServoIndexOffset+1]));
                                        
    servoStates[legServoIndexOffset+2] = servoStates[legServoIndexOffset+2] + (maxSwingRate*(controller[5] - controller[4]) / MIDSTATE) * 
                                        ((maxServoStates[legServoIndexOffset+2] - minServoStates[legServoIndexOffset+2]) / 
                                        abs(maxServoStates[legServoIndexOffset+2] - minServoStates[legServoIndexOffset+2]));
    */
  
    prevServo = currentTime;
  }
}


// Function to write servoState to all servos
void writeStatesToServos(){
  for (int k = 0; k < SERVOS; k ++){
    // Clamp servoStates within limits
    if (maxServoStates[k] > minServoStates[k]){
      if (servoStates[k] > maxServoStates[k]){
        servoStates[k] = maxServoStates[k];
      }
      else if (servoStates[k] < minServoStates[k]){
        servoStates[k] = minServoStates[k];
      }
    }
    else {
      if (servoStates[k] > minServoStates[k]){
        servoStates[k] = minServoStates[k];
      }
      else if (servoStates[k] < maxServoStates[k]){
        servoStates[k] = maxServoStates[k];
      }
    }
    
    
    // Write state to servo
    joints[k].writeMicroseconds(servoStates[k]);
  }
}


// Function to write servoStates to serial
void writeServoStateSerial() {
  Serial.print("<s");
  
  Serial.print(" ");
  Serial.print(int(currentLeg));
  Serial.print(" |");
  
  if (true) {
    for (int i = (3*currentLeg); i < ((3*currentLeg)+3); i++) {
      Serial.print(" ");
      Serial.print(int(servoStates[i]));
    }
    
    Serial.print(" |");
  }
  
  if (true) {
    for (int i = (3*currentLeg); i < ((3*currentLeg)+3); i++) {
      Serial.print(" ");
      Serial.print(int(servoAngles[i]));
    }
  }
  
  Serial.println(">");
}


// Function to check for leg change
void checkLegChange() {
  static boolean legChange = false;
  
  if ((controller[6]==1)&&(legChange==false)) {
    legChange = true;
    currentLeg = currentLeg + 1;
    
    if (currentLeg > (LEGS - 1)) {
      currentLeg = 0;
    }
  }
  
  else if ((controller[6]==0)&&(legChange==true)) {
    legChange = false;
  }
}
