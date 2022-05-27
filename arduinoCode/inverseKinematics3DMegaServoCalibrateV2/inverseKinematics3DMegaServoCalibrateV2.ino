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
#define STATES 7
#define MIDSTATE 10

// L3x2, R3x2, L2, R2, Square
int controller[STATES] = {10, 10, 10, 10, 0, 0, 0};
int currentLeg = 0;
int legIndexOffset = 3*currentLeg;
int legServoIndexOffset = 3*currentLeg;


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
  
int minServoStates[SERVOS] = {1000, 1000, 1000,
                              1000, 1000, 1000,
                              1175, 1825,  925,
                              1835,  915, 1725};
                              
int maxServoStates[SERVOS] = {2000, 2000, 2000,  
                              2000, 2000, 2000,
                              1825,  950, 1815,
                              1165, 1925, 780};
                              
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

  // Setting up servos
  attachServoPins();

  // Scaling to kinematics time step
  maxEndpointVelocity = maxEndpointVelocity * kinematicsRefreshTime / 1000;

  // Updating previous state arrays
  updatePrevServoStates();
  updatePrevLegEndpointPosition();
  
  currentTime = millis();
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
    // Write servo and kinematics states to serial
    writeStatesSerial();
  } 

  // IK loop
  if ((currentTime - prevKinematic) >= kinematicsRefreshTime) {
    // Update previous kinematics time step
    prevKinematic = currentTime;

    updateLegEndpointPosition();

    /* 
     * IK algorithm begin 
     */
    
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
    for (int i = legIndexOffset; i < legIndexOffset+3; i++) {
      legAngles[i] = legAngles[i]*180/PI;
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
    
    // Print values to serial (for testing)
    printDebug();
  }
  // IK loop end
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


// Function to attach pins to corresponding servos
void attachServoPins() {
  for (int i = 0; i < SERVOS; i ++){  
    // Attach pins to the corresponding servo
    joints[i].writeMicroseconds(servoStates[i]);
    joints[i].attach(servoPins[i]);
  
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
    for (int i = 0; i < STATES; i ++){
      controller[i] = atoi(token);
      
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

    legIndexOffset = 3 * currentLeg;
  }
  
  else if ((controller[6]==0)&&(legChange==true)){
    legChange = false;
  }
}


void updateLegEndpointPosition() {
  legEndpointPosition[legIndexOffset+0] = legEndpointPosition[legIndexOffset+0] + 
            maxEndpointVelocity * (controller[1] - MIDSTATE) / MIDSTATE;
  legEndpointPosition[legIndexOffset+1] = legEndpointPosition[legIndexOffset+1] + 
            maxEndpointVelocity * (controller[5] - controller[4]) / MIDSTATE;
  legEndpointPosition[legIndexOffset+2] = legEndpointPosition[legIndexOffset+2] + 
            maxEndpointVelocity * (controller[0] - MIDSTATE) / MIDSTATE;
}


// Function to map IK results to servos and their calibrations
void mapAnglesToServoAngles() {
  for (int i = 0; i < SERVOS; i++) {
    // Clamping angles to min and max values
    if (legAngles[i] < minLegAngles[i]) {
      legAngles[i] = minLegAngles[i];
    }
    else if (legAngles[i] > maxLegAngles[i]) {
      legAngles[i] = maxLegAngles[i];
    }
    
    servoAngles[i] = legAngles[i] - minLegAngles[i];
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
  }
}


// Function to write servoState to all servos
void writeStatesToServos(){
  jointLimitsViolated = false;
  
  for (int k = 0; k < SERVOS; k ++) {
    // Clamp servoStates within limits
    if (maxServoStates[k] > minServoStates[k]){
      if (servoStates[k] > maxServoStates[k]){
        servoStates[k] = maxServoStates[k];
        jointLimitsViolated = true;
      }
      else if (servoStates[k] < minServoStates[k]){
        servoStates[k] = minServoStates[k];
        jointLimitsViolated = true;
      }
    }
    else {
      if (servoStates[k] > minServoStates[k]){
        servoStates[k] = minServoStates[k];
        jointLimitsViolated = true;
      }
      else if (servoStates[k] < maxServoStates[k]){
        servoStates[k] = maxServoStates[k];
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
    joints[k].writeMicroseconds(servoStates[k]);
  }
}


// Print endpoint position and leg angles
void printDebug() {
  if (verboseDebug) {
    Serial.print("Current leg: ");
    Serial.println(currentLeg);
    
    Serial.print("legEndpointPosition[0](stepLongitudinal): ");
    Serial.println(legEndpointPosition[0]);
      
    Serial.print("legEndpointPosition[1](groundHeight): ");
    Serial.println(legEndpointPosition[1]);

    Serial.print("legEndpointPosition[2](stepTransverse): ");
    Serial.println(legEndpointPosition[2]);

    Serial.print("shoulderFootProjectionLength: ");
    Serial.println(shoulderFootProjectionLength);

    Serial.print("shoulderFootLegLength: ");
    Serial.println(shoulderFootLegLength);

    Serial.print("hipFootProjectionLength: ");
    Serial.println(hipFootProjectionLength);

    Serial.print("cosKneeAngle: ");
    Serial.println(cosKneeAngle);

    Serial.print("shoulderFootEffectiveAngle: ");
    Serial.print(shoulderFootEffectiveAngle);
    Serial.print(" ");
    Serial.println(shoulderFootEffectiveAngle*180/PI);

    Serial.print("shoulderAngleSupplementary: ");
    Serial.print(shoulderAngleSupplementary);
    Serial.print(" ");
    Serial.println(shoulderAngleSupplementary*180/PI);

    Serial.print("legAngles[0](Hip): ");
    Serial.println(legAngles[0]);

    Serial.print("legAngles[1](Shoulder): ");
    Serial.println(legAngles[1]);

    Serial.print("legAngles[2](Knee): ");
    Serial.println(legAngles[2]);

    Serial.print("Time taken: ");
    Serial.print(millis()-prevKinematic);
    Serial.println(" ms");
      
    Serial.print("\n");
  }
}


void writeStatesSerial() {
  static boolean writeServo = true;
  static boolean writeLegAngles = true;
  static boolean writeServoAngles = true;
  static boolean writeKinematics = true;

  Serial.print("<");

  if (writeServo) {
    Serial.print("s");
    Serial.print(" ");
    Serial.print(int(currentLeg));
  
    for (int i = legIndexOffset; i < (legIndexOffset+3); i ++){
      Serial.print(" ");
      Serial.print(int(servoStates[i]));
    }
    Serial.print(" | ");
  }
  
  if (writeLegAngles) {
    Serial.print("l");
    Serial.print(" ");
    Serial.print(int(currentLeg));
  
    for (int i = legIndexOffset; i < (legIndexOffset+3); i ++){
      Serial.print(" ");
      Serial.print(int(legAngles[i]));
    }
    Serial.print(" | ");
  }
  
  if (writeServoAngles) {
    Serial.print("a");
    Serial.print(" ");
    Serial.print(int(currentLeg));
  
    for (int i = legIndexOffset; i < (legIndexOffset+3); i ++){
      Serial.print(" ");
      Serial.print(int(servoAngles[i]));
    }
    Serial.print(" | ");
  }

  if (writeKinematics) {
    Serial.print("k");
    Serial.print(" ");
    Serial.print(int(currentLeg));

    for (int i = legIndexOffset; i < (legIndexOffset+3); i++) {
      Serial.print(" ");
      Serial.print(int(legEndpointPosition[i]));
    }
  }

  Serial.println(">");
}


// Function to update previous servo state array
void updatePrevServoStates() {
  for (int i = 0; i < SERVOS; i++) {
    prevServoStates[i] = servoStates[i];
  }
}


// Function to update previous leg endpoint position array
void updatePrevLegEndpointPosition() {
  for (int i = 0; i < (3*LEGS); i++) {
    prevLegEndpointPosition[i] = legEndpointPosition[i];
  }
}


// Restore servo states from previous
void restorePrevServoStates() {
  for (int i = 0; i < SERVOS; i++) {
    servoStates[i] = prevServoStates[i];
  }
}


// Restore leg endpoint position from previous
void restorePrevLegEndpointPosition() {
  for (int i = 0; i < (3*LEGS); i++) {
    legEndpointPosition[i] = prevLegEndpointPosition[i];
  }
}
