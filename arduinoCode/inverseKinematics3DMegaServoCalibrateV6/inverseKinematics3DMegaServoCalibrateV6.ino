/*
 * Sit, stand, twerk implementation for O.R.B V2
 */

#include <Servo.h>
#include <math.h>
#include <avr/wdt.h>



// Serial recieve variables
#define BAUDRATE 115200
const byte numChars = 48;
char receivedChars[numChars];
boolean newData = false;
boolean serialConnected = false;
char testWord[] = "ping";
unsigned int pingInterval = 500;


// Timing variables
unsigned int currentTime = 0;                  // ms
unsigned int prevKinematic = 0;                // ms

// Refresh times
unsigned int kinematicsRefreshTime = 25;       // ms
unsigned int sitKinematicsRefreshTime = 100;   // ms
unsigned int standKinematicsRefreshTime = 25;  // ms

// Transition timing
unsigned int transitionBeginTime = 0;
unsigned int standTransitionTime = 2500; 


// Controller state
// Number of independent controller state variables
#define STATES 11
#define MIDSTATE 50

// Controller variables
// L3x2, R3x2, L2, R2, Square, Cross, Triangle, Circle, Options
int controller[STATES] = {MIDSTATE, MIDSTATE, MIDSTATE, 
                          MIDSTATE,        0,        0,
                                 0,        0,        0,
                                 0,        0};
float filteredController[STATES];
// Controller filtering
boolean filterControllerStates = true;
float filterControllerWidth = 4;

// State machine control variables
boolean globalLegControl = false;
boolean move = false;     // Move if true, else stay and twerk
boolean stand = false;    // Stand if true, sit if false; if stand, IK enabled at setup()
boolean transition = false;


// Interpolations
// Transition interpolation
float interpolationFraction = 0;


// Leg and Servo variables
// Expands array as required
#define LEGS 4
#define SERVOS 12

/* 
Leg indices are defined as shown below:
        0      1
        
        
        2      3
*/

// Leg length parameters
float legLengths[LEGS] = {70,      // mm; length from hip to shoulder
                          150,     // mm; length from shoulder to knee
                          150,     // mm; length from knee to foot center
                          5   };   // mm; foot depth
                                                   
int legEnable[LEGS] = {0, 0, 0, 0};

// Leg indexing
int currentLeg = 0;
int legIndex = 0;
int legIndexOffset = 0;

float legOffsets[(3*LEGS)] = { 142.5, 0, -65,         //   Top
                               142.5, 0,  65,         //    x
                              -142.5, 0, -65,         //    |__z
                              -142.5, 0,  65};        // Position of each leg reference from robot center

float legOffsetsRotated[(3*LEGS)];
float legOffsetsRotatedTemp[(3*LEGS)];
                                

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
int minServoStates[SERVOS] = {1255, 1875,  775,
                              1750,  750, 2235,
                              1175, 1900,  900,
                              1725,  915, 1750};
                              
int maxServoStates[SERVOS] = {1915,  875, 1562,  
                              1120, 1715, 1250,
                              1825, 1025, 1790,
                              1055, 1925,  805};
                              
int jointRanges[SERVOS] = {60, 90, 90,
                           60, 90, 90,
                           60, 90, 90,
                           60, 90, 90};
                           
// Flags
boolean jointLimitsViolated = false;


// Leg endpoint positions; each leg has own reference
float legEndpointPosition[(3*LEGS)];

float prevLegEndpointPosition[(3*LEGS)];

float legEndpointStandInit[(3*LEGS)] = {00, 125, 85,  // mm; {back, down, outer}
                                        00, 125, 85,
                                        00, 125, 85,
                                        00, 125, 85};

float legEndpointStayInit[(3*LEGS)];

float maxEndpointVelocity = 150; // mm/s

// Setting leg endpoint limits
float maxLegEndpointPosition[3] = {150, 250, 120};

float minLegEndpointPosition[3] = {-150, 125, 50};


// Leg angle parameters
float legAngles[SERVOS] = {0, 0, 0,     // {Hip, shoulder, knee}
                           0, 0, 0,
                           0, 0, 0,   
                           0, 0, 0};

float minLegAngles[SERVOS] = {-20, 7.5, 37.5,
                              -20, 7.5, 37.5,
                              -20, 7.5, 37.5,
                              -20, 7.5, 37.5};
                              
float maxLegAngles[SERVOS] = {40, 97.5, 127.5,
                              40, 97.5, 127.5,
                              40, 97.5, 127.5,
                              40, 97.5, 127.5};

// Leg angles for sitting state                   
float legAnglesSit[SERVOS] = {0, 45, 25,     // {Hip, shoulder, knee}
                              0, 45, 25,
                              0, 45, 25,
                              0, 45, 25};

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


// Twerk control parameters and variables
// Yaw in x-z plane                            x
// Pitch in x-y plane                          |__ z
// Roll in z-y plane
float twerkAngles[3] = {0, 0, 0};
float twerkAngleLimits[3] = {25, 15, 15};       // [yaw, pitch, roll] degrees
float twerkTranslations[(3*LEGS)];              // [x, y, z] offsets in mm


// Gait / stride parameters
// Stride parameters
float strideHeight = 50;                  // [y] stride height offset at peak
float strideVelocity[3] = {0, 0, 0};            // [x, z, theta] robot centered linear and angular velocity

// Stride states
int legContact[LEGS] = {1, 1, 1, 1};      // 1 if in contact, 0 if lifted
float overallStrideCycle = 0;                    // Stride cycle state
float legStrideCycles = {0, 0, 0, 0};

// Key stride points
float overallStrideStart = 0;
float legStrideLift = 0;

// Leg positions at beginning of lift
float liftBeginEndpointPosition[(3*LEGS)];

// Limits
unsigned int strideTime = 3000;
unsigned int liftTime = 1000;
float maxVelocity[3] = {0, 0, 0};

// Timing
unsigned int strideBeginTime = 0;



// Setup function
void setup() { 
  // Establishing connection through serial
  establishSerialConnection();
  
  // Initializing controller filter
  initializeControllerFilter();

  // Initializing leg endpoint position
  setLegEndpointToStand();
  
  // Converting angle limits from degrees to radians
  for (int angle = 0; angle < 3; angle ++) {
    twerkAngleLimits[angle] = twerkAngleLimits[angle] * PI / 180;
  }
  
  // Initial evaluation of IK
  currentTime = millis();
  
  // Differing initial position based on whether standing or sitting
  // If stand
  if (stand) {
    // Selecting parameters
    setStandKinematicsParameters(); 
    
    // Evaluate IK to initialize
    evaluateInverseKinematics();
    
    // Map and write leg angles to servos
    writeAnglesToServos();
  }
  // If sit
  else if (!stand) {
    // Selecting parameters
    setSitKinematicsParameters();
    
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
    checkLegChange();
    checkGlobalLegControl();
    checkStand();
    checkMove();
    
    // Write servo and kinematics states to serial
    writeStatesSerial();
  }

  // Code below runs at kinematicsRefreshTime
  if ((currentTime - prevKinematic) >= kinematicsRefreshTime) {

    // Transitioning to stand state
    if ((stand) && (transition)) {
      if ((currentTime - transitionBeginTime) <= standTransitionTime) {
        // Update legAngles to stand angles
        setLegEndpointToStand();
        evaluateInverseKinematics();

        // Interpolate to stand
        interpolateStand();

        // Write interpolated leg angles to servos
        writeAnglesToServos();
      }
      // Transition to stand complete
      else {
        // Selecting parameters
        setStandKinematicsParameters();
        transition = false;

        // Initialize stand leg endpoints to potentially stay on
        storeLegEndpointOnStay();
      }
    } 

    // Stand IK loop
    if ((stand) && (!transition)) {
      // Update endpoint position with controller inputs
      updateLegEndpointPosition();
      // Evaluate leg angles
      evaluateInverseKinematics();
      // Write leg angles to servos
      writeAnglesToServos();

    }

    // Transitioning to sit state
    if ((!stand) && (transition)) {
      if ((currentTime - transitionBeginTime) <= standTransitionTime) {
        // Update legAngles to stand angles
        setLegAnglesToSit();

        // Interpolate to sit
        interpolateStand();

        // Write interpolated leg angles to servos
        writeAnglesToServos();
      } 
      // Transition to sit complete
      else {
        // Selecting parameters
        setSitKinematicsParameters();
        transition = false;
      }
    }

    // Sit loop
    if ((!stand) && (!transition)) {
      // Nothing to do here
    }
  } 
}


// Function to establish connection through serial
void establishSerialConnection() {
  Serial.begin(BAUDRATE);
  
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
  for (legIndex = 0; legIndex < LEGS; legIndex ++) {  
    legIndexOffset = legIndex * 3;

    // Attach pins to servos in leg
    for (int servoOffset = 0; servoOffset < 3; servoOffset ++) {
      joints[legIndexOffset+servoOffset].writeMicroseconds(servoStates[legIndexOffset+servoOffset]);
      
      // Attach pin to servo if leg enabled
      if (legEnable[legIndex]) {
        joints[legIndexOffset+servoOffset].attach(servoPins[legIndexOffset+servoOffset]);
        
        // Delay before continuing
        delay(25);
      }
    }
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


// Initialize controller filter
void initializeControllerFilter() {
  for (int state = 0; state < STATES; state ++) {
    filteredController[state] = controller[state];
  }
}


/* 
 * Function to extract controller values from serial message
 * on message start character
 * Also filters controller values if enabled
 */
void extractControllerState(){
  char * token = strtok(receivedChars, " ");
  
  if (*token == 'd'){
    token = strtok(NULL, " ");
    for (int state = 0; state < STATES; state ++){

      // Selectively filter analog states
      if ((filterControllerStates) && (state < 6)) {
        filteredController[state] = filteredController[state] + 
          ((float(atoi(token)) - filteredController[state]) / filterControllerWidth);

        controller[state] = int(filteredController[state]);
      } else {
        controller[state] = atoi(token);
      }
      
      token = strtok(NULL, " ");
    }
  }
}


// Set stand kinematics parameters
void setStandKinematicsParameters() {
  // Changing refresh time
  kinematicsRefreshTime = standKinematicsRefreshTime;
  // Scaling to kinematics time step
  maxEndpointVelocity = maxEndpointVelocity * kinematicsRefreshTime / 1000;
}


// Set sit kinematics parameters
void setSitKinematicsParameters() {
  // Changing refresh time
  kinematicsRefreshTime = sitKinematicsRefreshTime;
  // Scaling to kinematics time step
  maxEndpointVelocity = maxEndpointVelocity * kinematicsRefreshTime / 1000;
}


// Set legEndpointPosition to stand position
void setLegEndpointToStand() {
  for (int dim = 0; dim < (3*LEGS); dim ++) {
    legEndpointPosition[dim] = legEndpointStandInit[dim];
  }
}


// Store legEndpoints on stay state
void storeLegEndpointOnStay() {
  for (int dim = 0; dim < (3*LEGS); dim ++) {
    legEndpointStayInit[dim] = legEndpointPosition[dim];
  }
}


// Load stored stay legEndpoints
void loadLegEndpointsOnStay() {
  for (int dim = 0; dim < (3*LEGS); dim ++) {
    legEndpointPosition[dim] = legEndpointStayInit[dim];
  }
}


// Set leg angles to the sit position
void setLegAnglesToSit() {
  for (int servo = 0; servo < SERVOS; servo++) {
    legAngles[servo] = legAnglesSit[servo];
  }
}


// Initialize stride parameters
void initializeStrideParameters() {
  // Calculating max velocities
  maxVelocity[0] = (maxLegEndpointPosition[0] - minLegEndpointPosition[0]) / strideTime;
  maxVelocity[1] = (maxLegEndpointPosition[2] - minLegEndpointPosition[2]) / strideTime;
  maxVelocity[2] = twerkAngleLimits[0] / strideTime;

  // Calculating key points of stride cycle
  overallStrideStart = 1 / LEGS;
  legStrideLift = strideTime / (strideTime + liftTime);

  // Initializing stride begin time
  strideBeginTime = currentTime;
}
    

// Grab velocity state
void updateStrideVelocity() {
  strideVelocity[0] = maxVelocity[0] * float(controller[1] - MIDSTATE) / MIDSTATE;
  strideVelocity[1] = maxVelocity[1] * float(controller[0] - MIDSTATE) / MIDSTATE;
  strideVelocity[2] = maxVelocity[2] * float(controller[2] - MIDSTATE) / MIDSTATE;
}


// Function to detach servos and reset
void checkReset() {
  // On Options press
  if (controller[10]==1) {
    detachServoPins();
    wdt_enable(WDTO_250MS);
    
    while (true){
      // hahahahahaha
    }
  }
}


// Function to check for global leg control
void checkGlobalLegControl(){  
  static boolean globalLegControlChange = false;

  // On Circle press
  if ((controller[9]==1) && (globalLegControlChange==false)){
    globalLegControlChange = true;
    
    if (globalLegControl) {
      globalLegControl = false;
    } else { 
      globalLegControl = true;
    }
  }
  
  else if ((controller[9]==0) && (globalLegControlChange==true)){
    globalLegControlChange = false;
  }
}


// Function to check for stand/sit
void checkStand() {
  static boolean standChange = false;
  
  // On Triangle press
  if ((controller[8]==1) && (standChange==false)){
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
  
  else if ((controller[8]==0) && (standChange==true)){
    standChange = false;
  } 
}


// Function to check for move change
void checkMove() {
  static boolean moveChange = false;

  // On Cross press
  if ((controller[7]==1) && (moveChange==false)) {
    moveChange = true;
    
    if (move) {
      // Stay mode
      move = false;

      // Store leg endpoints to stay on
      storeLegEndpointOnStay();

    } else {
      // Move mode
      move = true;

      // Initialize move mode and stride parameters
      initializeStrideParameters();
    }
  }
  
  else if ((controller[7]==0) && (moveChange==true)) {
    moveChange = false;
  }
}


// Function to check for leg change
void checkLegChange(){
  static boolean legChange = false;

  // On Square press
  if ((controller[6]==1) && (legChange==false)){
    legChange = true;
    currentLeg = currentLeg + 1;
    
    if (currentLeg > (LEGS - 1)) {
      currentLeg = 0;
    }
  }
  
  else if ((controller[6]==0) && (legChange==true)){
    legChange = false;
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
  // Hold leg endpoint positions or stay, aka twerk mode
  if ((stand) && (!move)) {
    // Load stored stay position
    loadLegEndpointsOnStay();

    // Globally control robot height
    for (legIndex = 0; legIndex < LEGS; legIndex++) {
      legIndexOffset = legIndex * 3;

      legEndpointPosition[legIndexOffset+1] = legEndpointPosition[legIndexOffset+1] + 
            maxEndpointVelocity * (controller[5] - controller[4]) / MIDSTATE;
      
      legEndpointStayInit[legIndexOffset+1] = legEndpointPosition[legIndexOffset+1];
    }
    
    // Yaw, pitch, roll
    twerkAngles[0] = twerkAngleLimits[0] * float(controller[2] - MIDSTATE) / MIDSTATE;
    twerkAngles[1] = twerkAngleLimits[1] * float(controller[1] - MIDSTATE) / MIDSTATE;
    twerkAngles[2] = twerkAngleLimits[2] * float(controller[0] - MIDSTATE) / MIDSTATE;

    for (legIndex = 0; legIndex < LEGS; legIndex ++) {
      legIndexOffset = legIndex * 3;

      // Computing leg positions after yaw (in x-z plane)
      // x
      legOffsetsRotated[legIndexOffset + 0] = 
                (legOffsets[legIndexOffset + 0] * cos(twerkAngles[0])) - 
                (legOffsets[legIndexOffset + 2] * sin(twerkAngles[0]));
      // y
      legOffsetsRotated[legIndexOffset + 1] =
                (legOffsets[legIndexOffset + 1]);
      // z
      legOffsetsRotated[legIndexOffset + 2] = 
                (legOffsets[legIndexOffset + 0] * sin(twerkAngles[0])) +
                (legOffsets[legIndexOffset + 2] * cos(twerkAngles[0]));
      
      // Computing leg positions after pitch (in x-y plane)
      // x
      legOffsetsRotatedTemp[legIndexOffset + 0] = 
                (legOffsetsRotated[legIndexOffset + 0] * cos(twerkAngles[1])) - 
                (legOffsetsRotated[legIndexOffset + 1] * sin(twerkAngles[1]));
      // y
      legOffsetsRotatedTemp[legIndexOffset + 1] = 
                (legOffsetsRotated[legIndexOffset + 0] * sin(twerkAngles[1])) + 
                (legOffsetsRotated[legIndexOffset + 1] * cos(twerkAngles[1]));
      // z
      legOffsetsRotatedTemp[legIndexOffset + 2] = 
                (legOffsetsRotated[legIndexOffset + 2]);

      // Computing leg positions after roll (in z-y plane)
      // x
      legOffsetsRotated[legIndexOffset + 0] = 
                (legOffsetsRotatedTemp[legIndexOffset + 0]);
      // y
      legOffsetsRotated[legIndexOffset + 1] = 
                (legOffsetsRotatedTemp[legIndexOffset + 2] * sin(twerkAngles[2])) +
                (legOffsetsRotatedTemp[legIndexOffset + 1] * cos(twerkAngles[2]));
      // z
      legOffsetsRotated[legIndexOffset + 2] = 
                (legOffsetsRotatedTemp[legIndexOffset + 2] * cos(twerkAngles[2])) -
                (legOffsetsRotatedTemp[legIndexOffset + 1] * sin(twerkAngles[2]));
    }

    // Finding per leg translation and translating leg endpoints
    for (legIndex = 0; legIndex < LEGS; legIndex ++) {
      legIndexOffset = 3 * legIndex;
      
      for (int dim = 0; dim < 3; dim ++) {
        twerkTranslations[legIndexOffset + dim] = 
          legOffsetsRotated[legIndexOffset + dim] - legOffsets[legIndexOffset + dim];

        if ((dim == 2) && ((legIndex == 3) || (legIndex == 1))) {
          legEndpointPosition[legIndexOffset + dim] = 
            legEndpointPosition[legIndexOffset + dim] - twerkTranslations[legIndexOffset + dim];
        } else {
          legEndpointPosition[legIndexOffset + dim] = 
            legEndpointPosition[legIndexOffset + dim] + twerkTranslations[legIndexOffset + dim];
        }
      }
    }
  }

  // Walk / move cycle
  else if ((stand) && (move)) {
    // Stride cycle state update
    overallStrideCycle = float(currentTime - strideBeginTime) / 
                  float(strideTime + liftTime);

    // Rollover to next cycle if overflow
    if (overallStrideCycle >= 1) {
      overallStrideCycle = overallStrideCycle - 1;
      
      // Update stride begin time
      strideBeginTime = currentTime;

      // Update stride velocities applicable for this stride
      updateStrideVelocity();

      // Generate leg stride start positions with stride velocities
      /*

      To implement

      */
    }

    // Updating legStrideCycles with overallStrideCycle
    for (legIndex = 0; legIndex < LEGS; legIndex ++) {
      legStrideCycles[legIndex] = overallStrideCycle - (overallStrideStart * legIndex);

      if (legStrideCycles[legIndex] < 0) {
        legStrideCycles[legIndex] = 1 + legStrideCycles[legIndex];
      }
      else if (legStrideCycles[legIndex] >= 1) {
        legStrideCycles[legIndex] = legStrideCycles[legIndex] - 1;
      }
    }
    // Leg stride cycle states updated

    // Flag each leg for lift at lift key stride points
    for (legIndex = 0; legIndex < LEGS; legIndex ++) {
      legIndexOffset = 3 * legIndex;

      if ((legStrideCycles[legIndex] >= legStrideLift) && (legContact[legIndex] == 1)) {
        legContact[legIndex] = 0;
        
        // Store position of leg endpoint at lift begin
        for (int dim = 0; dim < 3; dim ++) {
          liftBeginEndpointPosition[legIndexOffset + dim] = 
                                              legEndpointPosition[legIndexOffset + dim];
        }
      }
    }

    // Transition from lift begin to stride start position for lifted legs
    /*
    
    To implement

    */

    // Move planted leg endpoints according to stride velocities
    /*

    To implement

    */
  }

  else if ((stand) && (!globalLegControl)){
    legIndexOffset = currentLeg * 3;
    
    legEndpointPosition[legIndexOffset+0] = legEndpointPosition[legIndexOffset+0] + 
            maxEndpointVelocity * (controller[1] - MIDSTATE) / MIDSTATE;
    legEndpointPosition[legIndexOffset+1] = legEndpointPosition[legIndexOffset+1] + 
            maxEndpointVelocity * (controller[5] - controller[4]) / MIDSTATE;
    legEndpointPosition[legIndexOffset+2] = legEndpointPosition[legIndexOffset+2] + 
            maxEndpointVelocity * (controller[0] - MIDSTATE) / MIDSTATE;
  }

  else if ((stand) && (globalLegControl)) {
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
    legIndexOffset = 3 * legIndex;
  
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
  // Flag for joint limit violation
  jointLimitsViolated = false;
  
  // Check if joint limits violated
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
  }

  // Use previous states if joint limits violated
  if (jointLimitsViolated) {
    restorePrevServoStates();
    restorePrevLegEndpointPosition();
  }

  // Update previous states if no violations
  else if (!jointLimitsViolated) {
    updatePrevServoStates();
    updatePrevLegEndpointPosition();

    // Write current servo states to servos
    for (int servo = 0; servo < SERVOS; servo ++) {
      // Write state to servo
      joints[servo].writeMicroseconds(servoStates[servo]);
    }
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
    if ((legEndpointPosition[legIndexOffset+2] - legLengths[0]) > 0) {
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
    
    if (legEndpointPosition[(3*legIndex)+0] < 0) {
      shoulderFootEffectiveAngle = PI - shoulderFootEffectiveAngle;
    }
    
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
  static boolean writeLeg = true;
  static boolean writeStates = true;
  
  legIndexOffset = 3 * currentLeg;

  Serial.print("<");

  if (writeServo) {
    Serial.print("s ");
  
    for (int servo = legIndexOffset; servo < (legIndexOffset+3); servo++){
      Serial.print(" ");
      Serial.print(int(servoStates[servo]));
    }
    Serial.print(" | ");
  }
  
  if (writeLegAngles) {
    Serial.print("l ");
  
    for (int servo = legIndexOffset; servo < (legIndexOffset+3); servo++){
      Serial.print(" ");
      Serial.print(int(legAngles[servo]));
    }
    Serial.print(" | ");
  }
  
  if (writeServoAngles) {
    Serial.print("a ");
  
    for (int servo = legIndexOffset; servo < (legIndexOffset+3); servo++){
      Serial.print(" ");
      Serial.print(int(servoAngles[servo]));
    }
    Serial.print(" | ");
  }

  if (writeKinematics) {
    Serial.print("k ");

    for (int dim = legIndexOffset; dim < (legIndexOffset+3); dim++) {
      Serial.print(" ");
      Serial.print(int(legEndpointPosition[dim]));
    }
    Serial.print(" | ");
  }

  if (writeLeg) {
    Serial.print("leg ");
    Serial.print(currentLeg);

    if (legEnable[currentLeg]) {
      Serial.print(" en ");
    } else {
      Serial.print(" na ");
    }
    Serial.print("| ");
  }

  if (globalLegControl) {
    Serial.print("global ");
  }
  
  if (writeStates) {
    if (stand) {
      Serial.print("stand ");
    } else {
      Serial.print("sit ");
    }

    if (transition) {
      Serial.print("transition ");
      Serial.print(interpolationFraction);
      Serial.print(" ");
    }

    if (move) {
      Serial.print("move ");
    } else {
      Serial.print("stay ");
      Serial.print(twerkTranslations[legIndexOffset + 0]);
      Serial.print(" ");
      Serial.print(twerkTranslations[legIndexOffset + 1]);
      Serial.print(" ");
      Serial.print(twerkTranslations[legIndexOffset + 2]);
    }
  }

  Serial.println(">");
}
