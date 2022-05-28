/*
 * Reference implementation of 3D Inverse Kinematics for a legged robot
 */

#include <Servo.h>
#include <math.h>


// Leg length parameters
float legLengths[4] = {70,      // mm; length from hip to shoulder
                       150,     // mm; length from shoulder to knee
                       150,     // mm; length from knee to foot center
                       5        // mm; foot depth
                       };
// Note that the parameters above are currently inaccurate


// Timing
unsigned int currentTime = 0;               // ms
unsigned int prevKinematic = 0;             // ms
unsigned int kinematicsRefreshTime = 20;    // ms


// Flags
bool verboseDebug = false;


// Serial recieve variables
#define BAUDRATE 115200
const byte numChars = 48;
char receivedChars[numChars];
boolean newData = false;
boolean serialConnected = false;
char testWord[] = "ping";
unsigned int pingInterval = 500;


// Controller state
// Number of independent controller state variables
#define STATES 8
#define MIDSTATE 10

// L3x2, R3x2, L2, R2, Square
int controller[STATES] = {10, 10, 10, 10, 0, 0, 0, 0};
int currentLeg = 0;
int legIndex = 0;
int legIndexOffset = 0;

boolean globalLegControl = false;


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
                             
float prevServoStates[SERVOS];

float servoAngles[SERVOS] = {0, 30, 0,    // Hip, shoulder, knee
                             0, 30, 0,
                             0, 30, 0,
                             0, 30, 0};

// Range of movement = {60, 90, 120} degrees for {hip, shoulder, knee}
  
int minServoStates[SERVOS] = {1125, 1845,  775,
                              1795,  780, 2235,
                              1175, 1825,  900,
                              1835,  915, 1750};
                              
int maxServoStates[SERVOS] = {1785,  840, 1800,  
                              1165, 1745, 1250,
                              1825,  950, 1790,
                              1165, 1925,  805};
                              
int jointRanges[SERVOS] = {60, 90, 90,
                           60, 90, 90,
                           60, 90, 90,
                           60, 90, 90};
                           
// Max swing rate in degrees per second
float maxSwingRate = 90;
// Flags
boolean jointLimitsViolated = false;


// Leg endpoint position; each leg has own reference
float legEndpointPosition[3*(LEGS)] = {000, 125, 75,  // mm; {back, down, outer}
                                       000, 125, 75,
                                       000, 125, 75,
                                       000, 125, 75};
float prevLegEndpointPosition[3*(LEGS)];
float maxEndpointVelocity = 100; // mm/s


// Leg angle parameters
float legAngles[SERVOS] = {0, 0, 0,     // {Hip, shoulder, knee}
                           0, 0, 0,
                           0, 0, 0,     // {Hip, shoulder, knee}
                           0, 0, 0};         
float minLegAngles[SERVOS] = {-20, 15, 30,
                              -20, 15, 30,
                              -20, 15, 30,
                              -20, 15, 30};
float maxLegAngles[SERVOS] = {40, 105, 120,
                              40, 105, 120,
                              40, 105, 120,
                              40, 105, 120};


// Effective and corrected parameters
// Lengths
float shoulderFootProjectionLength = 0;
float shoulderFootLegLength = 0;
float hipFootProjectionLength = 0;
// Angles
float shoulderFootEffectiveAngle = 0;
// Parameters
float cosKneeAngle = 0;
float cosKneeAngleSupplementary = 0;
float sinKneeAngleSupplementary = 0;
float shoulderAngleSupplementary = 0;



void setup() {
  // Establishing connection through serial
  establishSerialConnection();
  
  // Scaling to kinematics time step
  maxEndpointVelocity = maxEndpointVelocity * kinematicsRefreshTime / 1000;
  
  // Initial evaluation of IK
  currentTime = millis();
  evaluateInverseKinematics();
  
  // Setting up servos
  attachServoPins();

  // Updating previous state arrays
  updatePrevServoStates();
  updatePrevLegEndpointPosition();
}


void loop() {
  // Update loop time
  currentTime = millis();

  // Recieve data from serial
  receiveSerialData();
  if (newData){
    newData = false;
    extractControllerState();
    checkLegChange();
    checkGlobalLegControl();
    // Write servo and kinematics states to serial
    writeStatesSerial();
  } 

  // IK loop
  if ((currentTime - prevKinematic) >= kinematicsRefreshTime) {
    
    updateLegEndpointPosition();
    evaluateInverseKinematics();

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
      
      for (int charIndex = 0; receivedChars[charIndex]!='\0'; charIndex++) {
        if (receivedChars[charIndex]!=testWord[charIndex]) {
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


// Function to attach pins to corresponding servos
void attachServoPins() {
  for (int servo = 0; servo < SERVOS; servo ++){  
    // Attach pins to the corresponding servo
    joints[servo].writeMicroseconds(servoStates[servo]);
    joints[servo].attach(servoPins[servo]);
  
    // Delay before continuing
    delay(100);
  }
}


// Function to remove limiters from serial message
void receiveSerialData() {
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
 * Function to extract controller values from serial message
 * on message start character
 */
void extractControllerState(){
  char * token = strtok(receivedChars, " ");
  
  if (*token == 'd'){
    token = strtok(NULL, " ");
    for (int state = 0; state < STATES; state ++){
      controller[state] = atoi(token);
      
      token = strtok(NULL, " ");
    }
  }
}


// Function to check for leg change
void checkLegChange(){
  static boolean legChange = false;

  if ((controller[6]==1)&&(legChange==false)){
    legChange = true;
    currentLeg = currentLeg + 1;
    
    if (currentLeg > (LEGS - 1)) {
      currentLeg = 0;
    }
  }
  
  else if ((controller[6]==0)&&(legChange==true)){
    legChange = false;
  }
}


// Function for checking for global leg control
void checkGlobalLegControl(){  
  static boolean globalLegControlChange = false;

  if ((controller[7]==1)&&(globalLegControlChange==false)){
    globalLegControlChange = true;
    
    if (globalLegControl) {
      globalLegControl = false;
    }
    else { 
      globalLegControl = true;
    }
  }
  
  else if ((controller[7]==0)&&(globalLegControlChange==true)){
    globalLegControlChange = false;
  }
}


// Function to update leg endpoint position based on controller input
void updateLegEndpointPosition() {
  if (globalLegControl) {
    for (legIndex = 0; legIndex < LEGS; legIndex++) {
      legIndexOffset = legIndex * 3;
    
      legEndpointPosition[legIndexOffset+0] = legEndpointPosition[legIndexOffset+0] + 
            maxEndpointVelocity * (controller[1] - MIDSTATE) / MIDSTATE;
      legEndpointPosition[legIndexOffset+1] = legEndpointPosition[legIndexOffset+1] + 
            maxEndpointVelocity * (controller[5] - controller[4]) / MIDSTATE;
      legEndpointPosition[legIndexOffset+2] = legEndpointPosition[legIndexOffset+2] + 
            maxEndpointVelocity * (controller[0] - MIDSTATE) / MIDSTATE;
    }
  }
  else {
    legIndexOffset = currentLeg * 3;
    
    legEndpointPosition[legIndexOffset+0] = legEndpointPosition[legIndexOffset+0] + 
            maxEndpointVelocity * (controller[1] - MIDSTATE) / MIDSTATE;
    legEndpointPosition[legIndexOffset+1] = legEndpointPosition[legIndexOffset+1] + 
            maxEndpointVelocity * (controller[5] - controller[4]) / MIDSTATE;
    legEndpointPosition[legIndexOffset+2] = legEndpointPosition[legIndexOffset+2] + 
            maxEndpointVelocity * (controller[0] - MIDSTATE) / MIDSTATE;
  }
}


// Function to map IK results to servos and their calibrations
void mapAnglesToServoAngles() {
  for (int servo = 0; servo < SERVOS; servo++) {
    // Clamping angles to min and max values
    if (legAngles[servo] < minLegAngles[servo]) {
      legAngles[servo] = minLegAngles[servo];
    }
    else if (legAngles[servo] > maxLegAngles[servo]) {
      legAngles[servo] = maxLegAngles[servo];
    }
    
    servoAngles[servo] = legAngles[servo] - minLegAngles[servo];
  }
}


// Function to update servo states
void updateServoStates(){
  for (legIndex = 0; legIndex < LEGS; legIndex++) {
    legIndexOffset = 3*legIndex;
  
    servoStates[legIndexOffset+0] = minServoStates[legIndexOffset+0] + (servoAngles[legIndexOffset+0] *
                                        (maxServoStates[legIndexOffset+0] - minServoStates[legIndexOffset+0]) / 
                                        jointRanges[legIndexOffset+0]);
                                        
    servoStates[legIndexOffset+1] = minServoStates[legIndexOffset+1] + (servoAngles[legIndexOffset+1] *
                                        (maxServoStates[legIndexOffset+1] - minServoStates[legIndexOffset+1]) / 
                                        jointRanges[legIndexOffset+1]);
   
    servoStates[legIndexOffset+2] = minServoStates[legIndexOffset+2] + (servoAngles[legIndexOffset+2] *
                                        (maxServoStates[legIndexOffset+2] - minServoStates[legIndexOffset+2]) / 
                                        jointRanges[legIndexOffset+2]);
  }
}


// Function to write servoState to all servos
void writeStatesToServos(){
  jointLimitsViolated = false;
  
  for (int servo = 0; servo < SERVOS; servo ++) {
    // Clamp servoStates within limits
    if (maxServoStates[servo] > minServoStates[servo]){
      if (servoStates[servo] > maxServoStates[servo]){
        servoStates[servo] = maxServoStates[servo];
        jointLimitsViolated = true;
      }
      else if (servoStates[servo] < minServoStates[servo]){
        servoStates[servo] = minServoStates[servo];
        jointLimitsViolated = true;
      }
    }
    else {
      if (servoStates[servo] > minServoStates[servo]){
        servoStates[servo] = minServoStates[servo];
        jointLimitsViolated = true;
      }
      else if (servoStates[servo] < maxServoStates[servo]){
        servoStates[servo] = maxServoStates[servo];
        jointLimitsViolated = true;
      }
    }
    
    // Use previous states if joint limits violated
    if (jointLimitsViolated) {
      restorePrevServoStates();
      restorePrevLegEndpointPosition();

      break;
    }
    else {
      updatePrevServoStates();
      updatePrevLegEndpointPosition();
    }
    
    // Write state to servo
    joints[servo].writeMicroseconds(servoStates[servo]);
  }
}


// Function to write states to serial
void writeStatesSerial() {
  static boolean writeServo = true;
  static boolean writeLegAngles = true;
  static boolean writeServoAngles = true;
  static boolean writeKinematics = true;
  
  legIndexOffset = 3 * currentLeg;

  Serial.print("<");
  
  if (globalLegControl) {
    Serial.print("global ");
  }

  if (writeServo) {
    Serial.print("s ");
    Serial.print(int(currentLeg));
  
    for (int servo = legIndexOffset; servo < (legIndexOffset+3); servo++){
      Serial.print(" ");
      Serial.print(int(servoStates[servo]));
    }
    Serial.print(" | ");
  }
  
  if (writeLegAngles) {
    Serial.print("l ");
    Serial.print(int(currentLeg));
  
    for (int servo = legIndexOffset; servo < (legIndexOffset+3); servo++){
      Serial.print(" ");
      Serial.print(int(legAngles[servo]));
    }
    Serial.print(" | ");
  }
  
  if (writeServoAngles) {
    Serial.print("a ");
    Serial.print(int(currentLeg));
  
    for (int servo = legIndexOffset; servo < (legIndexOffset+3); servo++){
      Serial.print(" ");
      Serial.print(int(servoAngles[servo]));
    }
    Serial.print(" | ");
  }

  if (writeKinematics) {
    Serial.print("k ");
    Serial.print(int(currentLeg));

    for (int dim = legIndexOffset; dim < (legIndexOffset+3); dim++) {
      Serial.print(" ");
      Serial.print(int(legEndpointPosition[dim]));
    }
  }

  Serial.println(">");
}


// Function to update previous servo state array
void updatePrevServoStates() {
  for (int servo = 0; servo < SERVOS; servo++) {
    prevServoStates[servo] = servoStates[servo];
  }
}


// Function to update previous leg endpoint position array
void updatePrevLegEndpointPosition() {
  for (int dim = 0; dim < (3*LEGS); dim++) {
    prevLegEndpointPosition[dim] = legEndpointPosition[dim];
  }
}


// Restore servo states from previous
void restorePrevServoStates() {
  for (int servo = 0; servo < SERVOS; servo++) {
    servoStates[servo] = prevServoStates[servo];
  }
}


// Restore leg endpoint position from previous
void restorePrevLegEndpointPosition() {
  for (int dim = 0; dim < (3*LEGS); dim++) {
    legEndpointPosition[dim] = prevLegEndpointPosition[dim];
  }
}


// Inverse kinematics algorithm
void evaluateInverseKinematics(){
  prevKinematic = currentTime;
    
  /* 
   * IK algorithm begin 
   */
  
  for (legIndex = 0; legIndex < LEGS; legIndex++) {
    legIndexOffset = legIndex * 3;
    // Solves IK only for current leg
    // Solving for hip angle
    hipFootProjectionLength = sqrt(pow(legEndpointPosition[legIndexOffset+1], 2) + 
                                   pow(legEndpointPosition[legIndexOffset+2], 2));
    shoulderFootProjectionLength = sqrt(pow(hipFootProjectionLength, 2) - 
                                        pow(legLengths[0], 2));
                                        
    // Leg length; shoulder to foot
    shoulderFootLegLength = sqrt(pow(legEndpointPosition[legIndexOffset+0], 2) + 
                                 pow(shoulderFootProjectionLength, 2));

    // Hip angle
    legAngles[legIndexOffset+0] = acos((legEndpointPosition[legIndexOffset+2] * legLengths[0] + 
                         legEndpointPosition[legIndexOffset+1] * shoulderFootProjectionLength) / 
                         pow(hipFootProjectionLength, 2));
    // Make angle negative for downward hip movement
    if ((legEndpointPosition[legIndexOffset+2] - legLengths[0]) < 0) {
      legAngles[legIndexOffset+0] = -legAngles[legIndexOffset+0];
    }
    // Hip angle computed

    // Solving for knee angle
    cosKneeAngle = (pow(legLengths[1], 2) + pow(legLengths[2], 2) - pow(shoulderFootLegLength, 2)) / 
                   (2 * legLengths[1] * legLengths[2]);
    // Knee angle
    legAngles[legIndexOffset+2] = acos(cosKneeAngle);
    // Knee angle computed

    // Solving for shoulder angle
    shoulderFootEffectiveAngle = atan2(pow(shoulderFootProjectionLength, 2),
                                       pow(legEndpointPosition[legIndexOffset+0], 2));
    cosKneeAngleSupplementary = - cosKneeAngle;
    sinKneeAngleSupplementary = sqrt(1 - pow(cosKneeAngleSupplementary, 2));
    shoulderAngleSupplementary = atan2(legLengths[2] * sinKneeAngleSupplementary, 
                                       legLengths[1] + legLengths[2] * cosKneeAngleSupplementary);
    // Shoulder angle                                  
    legAngles[legIndexOffset+1] = shoulderFootEffectiveAngle - shoulderAngleSupplementary;
    // Shoulder angle computed

    // All angles are in rad; converting to deg
    for (int servo = legIndexOffset; servo < legIndexOffset+3; servo++) {
      legAngles[servo] = legAngles[servo]*180/PI;
    }
  }
  
  /*
   * IK algorithm end
   */
   
  // The following run at the same rate as IK, no reason to run faster

  // Map IK results to servoStates
  mapAnglesToServoAngles();
  updateServoStates();
    
  // Update and write computed servo positions
  writeStatesToServos();
}
