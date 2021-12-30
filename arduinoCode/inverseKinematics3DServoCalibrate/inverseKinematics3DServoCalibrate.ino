/*
 * Reference implementation of 3D Inverse Kinematics for a legged robot
 */

#include <Servo.h>
#include <math.h>


// Leg length parameters
float legLengths[4] = {55,      // mm; length from hip to shoulder
                       170,     // mm; length from shoulder to knee
                       170,     // mm; length from knee to foot center
                       5        // mm; foot depth
                       };
// Note that the parameters above are currently inaccurate


// Timing
unsigned int currentTime = 0;               // ms
unsigned int prevKinematic = 0;             // ms
unsigned int kinematicsRefreshTime = 100;    // ms


// Flags
bool verboseDebug = true;


// Serial recieve variables
#define BAUDRATE 115200
const byte numChars = 48;
char receivedChars[numChars];
bool newData = false;
bool serialConnected = false;
char testWord[] = "ping";
unsigned int serialConnInterval = 500;


// Controller state
// Number of independent controller state variables
#define STATES 7
#define MIDSTATE 10

// L3x2, R3x2, L2, R2, Square
int controller[STATES] = {10, 10, 10, 10, 0, 0, 0};
int currentLeg = 0;
int legIndexOffset = 3*currentLeg;


// Servo variables
// Expands array as required
#define LEGS 1
#define SERVOS 6

Servo joints[SERVOS];
int servoPins[SERVOS] = {5, 3,  6,
                         9, 10, 11};
// Initially set to positions specified below
float servoStates[SERVOS] = {115, 90, 15,      // Hip, shoulder, knee
                             40,  70, 162};
// Range of movement = {60, 90, 135} degrees for {hip, shoulder, knee}
int maxServoStates[SERVOS] = {75, 0,   150,  
                              0,  160, 27};  
int minServoStates[SERVOS] = {135, 90, 15,
                              60,  70, 162};
// Max swing rate in degrees per second
float maxSwingRate = 90;


// Leg endpoint position; each leg has own reference
float legEndpointPosition[3*(LEGS+1)] = {100, 200, 75,  // mm; {back, down, outer}
                                         100, 200, 75}; 
float maxEndpointVelocity = 50; // mm/s


// Leg angle parameters
float legAngles[SERVOS] = {0, 0, 0,     // {Hip, shoulder, knee}
                           0, 0, 0};         
float minLegAngles[SERVOS] = {-20, 0, 30,
                              -20, 0, 30};
float maxLegAngles[SERVOS] = {40, 90, 165,
                              40, 90, 165};


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
  
  currentTime = millis();
}


void loop() {
  // Update loop time
  currentTime = millis();

  // Recieve data from serial
  recieveSerialData();
  if (newData){
    newData = false;
    extractControllerState();
    checkLegChange();
    writeServoStateSerial();
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
    mapAnglesToServos();
    
    // Update and write computed servo positions
    writeStatesToServos();
    
    // Print values to serial (for testing)
    printDebug();

    // Write kinematics state to serial (for data display/logging)
    writeKinematicsStateSerial();
  }
  // IK loop end
}


// Function to establish connection through serial
void establishSerialConnection() {
  // Starting serial
  Serial.begin(BAUDRATE);

  while (!serialConnected) {
    Serial.print("<");
    Serial.print(testWord);
    Serial.println(">");
    delay(serialConnInterval);

    recieveSerialData();
  
    if (newData) {
      newData = false;
      serialConnected = true; 

      for (int i = 0; receivedChars[i]!='\0'; i++) {
        if (receivedChars[i]!=testWord[i]) {
          serialConnected = false;
        
          continue;
        }
      }
    }
  }
}


// Function to attach pins to corresponding servos
void attachServoPins() {
  for (int i = 0; i < SERVOS; i ++){  
    // Attach pins to the corresponding servo
    joints[i].write(servoStates[i]);
    joints[i].attach(servoPins[i]);
  
    // Delay before continuing
    delay(500);
  }
}


// Function to remove start and stop characters from serial message
void recieveSerialData(){
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
    
    if (currentLeg > LEGS) {
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
void mapAnglesToServos() {
  for (int i = 0; i < SERVOS; i++) {
    // Clamping angles to min and max values
    if (legAngles[i] < minLegAngles[i]) {
      legAngles[i] = minLegAngles[i];
    }
    else if (legAngles[i] > maxLegAngles[i]) {
      legAngles[i] = maxLegAngles[i];
    }
    
    servoStates[i] = minServoStates[i] + 
                     (legAngles[i] - minLegAngles[i]) * 
                     (maxServoStates[i] - minServoStates[i]) / 
                     abs(maxServoStates[i] - minServoStates[i]);
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
    joints[k].write(servoStates[k]);
  }
}


// Function to write servoStates to serial
void writeServoStateSerial(){
  Serial.print("<s");
  
  Serial.print(" ");
  Serial.print(int(currentLeg));
  Serial .print(" ");
  
  for (int i = (3*currentLeg); i < ((3*currentLeg)+3); i ++){
    Serial.print(" ");
    Serial.print(int(servoStates[i]));
  }
  Serial.println(">");
}


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
    Serial.print(legAngles[0]);
    Serial.print(" ");
    Serial.println(legAngles[0]*180/PI);

    Serial.print("legAngles[1](Shoulder): ");
    Serial.print(legAngles[1]);
    Serial.print(" ");
    Serial.println(legAngles[1]*180/PI);

    Serial.print("legAngles[2](Knee): ");
    Serial.print(legAngles[2]);
    Serial.print(" ");
    Serial.println(legAngles[2]*180/PI);

    Serial.print("Time taken: ");
    Serial.print(millis()-prevKinematic);
    Serial.println(" ms");
      
    Serial.print("\n");
  }
}


void writeKinematicsStateSerial() {
  Serial.print("<k");

  Serial.print(" ");
  Serial.print(int(currentLeg));
  Serial.print(" ");

  for (int i = legIndexOffset; i < (legIndexOffset+3); i++) {
    Serial.print(" ");
    Serial.print(int(legEndpointPosition[i]));
  }

  Serial.println(">");
}
