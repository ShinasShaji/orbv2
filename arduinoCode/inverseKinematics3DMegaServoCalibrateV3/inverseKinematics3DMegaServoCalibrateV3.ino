/*
 * Reference implementation of 3D Inverse Kinematics for a legged robot
 */

#include <Servo.h>
#include <math.h>
#include <avr/wdt.h>


// Leg length parameters
float legLengths[4] = {70,      // mm; length from hip to shoulder
                       150,     // mm; length from shoulder to knee
                       150,     // mm; length from knee to foot center
                       5        // mm; foot depth
                       };
// Note that the parameters above are currently inaccurate


// Timing variables
unsigned int currentTime = 0;               // ms
unsigned int prevKinematic = 0;             // ms
unsigned int kinematicsRefreshTime = 20;    // ms


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
#define STATES 10
#define MIDSTATE 50

// Controller variables
// L3x2, R3x2, L2, R2, Square
int controller[STATES] = {MIDSTATE, MIDSTATE, MIDSTATE, MIDSTATE, 0, 0, 0, 0, 0, 0};
int currentLeg = 0;
int legIndex = 0;
int legIndexOffset = 0;

// Control states
boolean globalLegControl = false;
boolean stand = false;    // Stand if true, sit if false; if stand, IK enabled at setup()

boolean transition = false;

// Timing
unsigned int transitionBeginTime = 0;
unsigned int standTransitionTime = 10000; 

// Transition interpolation
float interpolationFraction = 0;


// Servo variables
// Expands array as required
#define LEGS 4
#define SERVOS 12

Servo joints[SERVOS];
int servoPins[SERVOS] = {13, 12, 11,
                         10,  9,  8,
                          7,  6,  5,
                          4,  3,  2};

// Initially set servos to positions specified below
float servoStates[SERVOS] = {1500, 1500, 1500,    // Hip, shoulder, knee
                             1500, 1500, 1500,
                             1500, 1500, 1500,
                             1500, 1500, 1500};
                              
float prevServoStates[SERVOS];

float servoAngles[SERVOS] = {20, 30, 0,    // Hip, shoulder, knee
                             20, 30, 0,
                             20, 30, 0,
                             20, 30, 0};

// Range of movement = {60, 90, 120} degrees for {hip, shoulder, knee}

int minServoStates[SERVOS] = {1255, 1845,  775,
                              1690,  780, 2235,
                              1175, 1825,  900,
                              1725,  915, 1750};
                              
int maxServoStates[SERVOS] = {1915,  840, 1800,  
                              1060, 1745, 1250,
                              1825,  950, 1790,
                              1055, 1925,  805};
                              
int jointRanges[SERVOS] = {60, 90, 90,
                           60, 90, 90,
                           60, 90, 90,
                           60, 90, 90};
                           
// Max swing rate in degrees per second
float maxSwingRate = 90;
// Flags
boolean jointLimitsViolated = false;


// Leg endpoint positions; each leg has own reference
float legEndpointPosition[3*(LEGS)];

float prevLegEndpointPosition[3*(LEGS)];

float legEndpointStandInit[3*(LEGS)] = {000, 125, 70,  // mm; {back, down, outer}
                                        000, 125, 70,
                                        000, 125, 70,
                                        000, 125, 70};

float maxEndpointVelocity = 100; // mm/s

// Setting leg endpoint limits
float maxLegEndpointPosition[3] = {150, 250, 120};

float minLegEndpointPosition[3] = {-50, 125,  50};


// Leg angle parameters
float legAngles[SERVOS] = {0, 0, 0,     // {Hip, shoulder, knee}
                           0, 0, 0,
                           0, 0, 0,   
                           0, 0, 0};      
float minLegAngles[SERVOS] = {-20, 15, 30,
                              -20, 15, 30,
                              -20, 15, 30,
                              -20, 15, 30};
                              
float maxLegAngles[SERVOS] = {40, 105, 120,
                              40, 105, 120,
                              40, 105, 120,
                              40, 105, 120};

// Leg angles for sitting state                   
float legAnglesSit[SERVOS] = {0, 45, 30,     // {Hip, shoulder, knee}
                              0, 45, 30,
                              0, 45, 30,
                              0, 45, 30};

// Leg angles at beginning of transition
float transitionBeginAngles[SERVOS];


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



// Setup function
void setup() { 
  // Establishing connection through serial
  establishSerialConnection();
  
  // Initializing leg endpoint position
  setLegEndpointToStand();
  
  // Scaling to kinematics time step
  maxEndpointVelocity = maxEndpointVelocity * kinematicsRefreshTime / 1000;
  
  // Initial evaluation of IK
  currentTime = millis();
  
  // Differing initial position based on whether standing or sitting
  if (stand) {
    // Evaluate IK to initialize
    evaluateInverseKinematics();
    
    // Map and write leg angles to servos
    writeAnglesToServos();
  }
  
  else if (!stand) {
    // Setting leg angles to sitting position
    setLegAnglesToSit();
    
    // Map and write leg angles to servos
    writeAnglesToServos();
  }
  
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
    
    // Check for and update flags
    checkReset();
    checkStand();
    checkLegChange();
    checkGlobalLegControl();
    
    // Write servo and kinematics states to serial
    writeStatesSerial();
  }

  // Code below runs at kinematicsRefreshTime
  if ((currentTime - prevKinematic) >= kinematicsRefreshTime) {

    // Transitioning to stand state
    if ((stand)&&(transition)) {
      if ((currentTime - transitionBeginTime) <= standTransitionTime) {
        // Update legAngles to stand angles
        setLegEndpointToStand();
        evaluateInverseKinematics();

        // Interpolate to stand
        interpolateStand();

        // Write interpolated leg angles to servos
        writeAnglesToServos();
      } else {
        transition = false;
      }
    } 

    // Stand IK loop
    if ((stand)&&(!transition)) {
    
      // Update endpoint position with controller inputs
      updateLegEndpointPosition();
      // Evaluate leg angles
      evaluateInverseKinematics();
      // Write leg angles to servos
      writeAnglesToServos();

    }

    // Transitioning to sit state
    if ((!stand)&&(transition)) {
      if ((currentTime - transitionBeginTime) <= standTransitionTime) {
        // Update legAngles to stand angles
        setLegAnglesToSit();

        // Interpolate to sit
        interpolateStand();

        // Write interpolated leg angles to servos
        writeAnglesToServos();
      } else {
        transition = false;
      }
    }

    // Sit loop
    if ((!stand)&&(!transition)) {
      // Nothing to do here
    }
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
    delay(25);
  }
}


//Function to detach pins from corresponding servos
void detachServoPins() {
  for (int servo = 0; servo < SERVOS; servo ++){  
    // Detach pins from the corresponding servo
    joints[servo].detach();
  
    // Delay before continuing
    delay(25);
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


// Set legEndpointPosition to stand position
void setLegEndpointToStand() {
  for (int dim = 0; dim < (3*LEGS); dim++) {
    legEndpointPosition[dim] = legEndpointStandInit[dim];
  }
}


// Set leg angles to the sit position
void setLegAnglesToSit() {
  for (int servo = 0; servo < SERVOS; servo++) {
    legAngles[servo] = legAnglesSit[servo];
  }
}
    

// Function to detach servos and reset
void checkReset() {
  if (controller[9]==1) {
    detachServoPins();
    wdt_enable(WDTO_250MS);
    
    while (true){
      // hahahahahaha
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


// Function to check for global leg control
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


// Function to check for stand/sit
void checkStand() {
  static boolean standChange = false;
  
  if ((controller[8]==1)&&(standChange==false)){
    standChange = true;
    
    if (stand) {
      stand = false;
      transition = true;
      transitionBeginTime = currentTime;

      // Copy angles at beginning of transition
      updateTransitionBeginAngles();

      // Set legAngles to sit angles
      setLegAnglesToSit();
      // Reinitialize leg endpoint to standing position
      setLegEndpointToStand();
    }
    else {
      stand = true;
      transition = true;
      transitionBeginTime = currentTime;

      // Copy angles at beginning of transition
      updateTransitionBeginAngles();

      // Set leg endpoint to stand position
      setLegEndpointToStand();
    }
  }
  
  else if ((controller[8]==0)&&(standChange==true)){
    standChange = false;
  } 
}


// Function to copy angles at transition beginning
void updateTransitionBeginAngles() {
  for (int servo = 0; servo < SERVOS; servo ++) {
    transitionBeginAngles[servo] = legAngles[servo];
  }
}


// Function to interpolate for stand transition
void interpolateStand() {
  // Interpolation fraction
  interpolationFraction = float(currentTime - transitionBeginTime) / 
                          float(standTransitionTime);

  // Interpolate from transition beginning to end leg angles
  for (int servo = 0; servo < SERVOS; servo ++) {
    legAngles[servo] = transitionBeginAngles[servo] + 
                     ((legAngles[servo] - transitionBeginAngles[servo]) * 
                       interpolationFraction); 
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
  
  // Limiting leg endpoint locations
  for (legIndex = 0; legIndex < LEGS; legIndex++) {
    legIndexOffset = legIndex * 3;
    
    for (int dim = 0; dim < 3; dim++) {
      if (legEndpointPosition[legIndexOffset+dim] > maxLegEndpointPosition[dim]) {
        legEndpointPosition[legIndexOffset+dim] = maxLegEndpointPosition[dim];
      }
      else if (legEndpointPosition[legIndexOffset+dim] < minLegEndpointPosition[dim]) {
        legEndpointPosition[legIndexOffset+dim] = minLegEndpointPosition[dim];
      }
    }
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


// Function to map leg angles and write as servo microseconds
void writeAnglesToServos() {
  // Map leg angles to servoStates
  mapAnglesToServoAngles();
  updateServoStates();
    
  // Update and write computed servo positions
  writeStatesToServos();
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
     
  // Map and write leg angles to servos
  writeAnglesToServos();
  
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


// Function to write states to serial
void writeStatesSerial() {
  static boolean writeServo = true;
  static boolean writeLegAngles = true;
  static boolean writeServoAngles = true;
  static boolean writeKinematics = true;
  static boolean writeStates = true;
  
  legIndexOffset = 3 * currentLeg;

  Serial.print("<");

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
    Serial.print(" | ");
  }

  if (globalLegControl) {
    Serial.print("global ");
  }
  
  if (writeStates) {
    if (stand) {
      Serial.print("stand ");
    }
    else {
      Serial.print("sit ");
    }

    if (transition) {
      Serial.print("transition ");
      Serial.print(interpolationFraction);
    }
  }

  Serial.println(">");
}
