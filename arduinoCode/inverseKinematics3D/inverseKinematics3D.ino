/*
 * Reference implementation of 3D Inverse Kinematics for a legged robot
 */

#include <Servo.h>
#include <math.h>


// Timing
unsigned int currentTime = 0;               // ms
unsigned int prevKinematic = 0;             // ms
unsigned int kinematicsRefreshTime = 100;    // ms

// Flags
bool verboseDebug = true;

// Leg length parameters
float legLengths[4] = {55,      // mm; length from hip to shoulder
                       170,     // mm; length from shoulder to knee
                       170,     // mm; length from knee to foot center
                       5        // mm; foot depth
                       };
// Note that the parameters above are currently inaccurate

// Leg endpoint position
float legEndpointPosition[3] = {100, 200, 75}; // mm; {back, down, outer}

// Leg angle parameters
float legAngles[3] = {0, 0, 0};         // {Hip, shoulder, knee}
float minLegAngles[3] = {-20, 0, 30};
float maxLegAngles[3] = {40, 90, 165};

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
  // Starting serial
  Serial.begin(115200);
  
  currentTime = millis();

}

void loop() {
  // Update loop time
  currentTime = millis();

  // Loops for serial data
  // legEndPointPosition is updated here

  if ((currentTime - prevKinematic) >= kinematicsRefreshTime) {
    // Update previous kinematics time step
    prevKinematic = currentTime;
    
    // Solving for hip angle
    hipFootProjectionLength = sqrt(pow(legEndpointPosition[1], 2) + 
                                   pow(legEndpointPosition[2], 2));
    shoulderFootProjectionLength = sqrt(pow(hipFootProjectionLength, 2) - 
                                        pow(legLengths[0], 2));
                                        
    // Leg length; shoulder to foot
    shoulderFootLegLength = sqrt(pow(legEndpointPosition[0], 2) + 
                                 pow(shoulderFootProjectionLength, 2));

    // Hip angle
    legAngles[0] = acos((legEndpointPosition[2]*legLengths[0] + 
                         legEndpointPosition[1]*shoulderFootProjectionLength) / 
                        pow(hipFootProjectionLength, 2));
    // Make angle negative for downward hip movement
    if ((legEndpointPosition[2]-legLengths[0]) < 0) {
      legAngles[0] = -legAngles[0];
    }
    // Hip angle computed

    // Solving for knee angle
    cosKneeAngle = (pow(legLengths[1], 2) + pow(legLengths[2], 2) - pow(shoulderFootLegLength, 2)) / 
                   (2 * legLengths[1] * legLengths[2]);
    // Knee angle
    legAngles[2] = acos(cosKneeAngle);
    // Knee angle computed

    // Solving for shoulder angle
    shoulderFootEffectiveAngle = atan2(pow(shoulderFootProjectionLength, 2),
                                       pow(legEndpointPosition[0], 2));
    cosKneeAngleSupplementary = - cosKneeAngle;
    sinKneeAngleSupplementary = sqrt(1 - pow(cosKneeAngleSupplementary, 2));
    shoulderAngleSupplementary = atan2(legLengths[2] * sinKneeAngleSupplementary, 
                                       legLengths[1] + legLengths[2] * cosKneeAngleSupplementary);
    // Shoulder angle                                  
    legAngles[1] = shoulderFootEffectiveAngle - shoulderAngleSupplementary;
    // Shoulder angle computed
                    
    // Print values to serial (for testing)
    printDebug();
  }
}


void printDebug() {
  if (verboseDebug) {
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
