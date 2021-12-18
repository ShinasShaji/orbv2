#include <Servo.h>
#include <math.h>


// Timing
unsigned int currentTime = 0;               // ms
unsigned int prevKinematic = 0;             // ms
unsigned int kinematicsRefreshTime = 1000;    // ms

// Leg length parameters
float legLengths[4] = {55,      // mm; length from hip to shoulder
                       170,     // mm; length from shoulder to knee
                       170,     // mm; length from knee to foot center
                       5        // mm; foot depth
                       };
// Note that the parameters above are currently inaccurate

// Leg endpoint position
float legEndPointPosition[3] = {100, 200, 55}; // mm; {back, down, outer}

// Leg angle parameters
float legAngles[3] = {0, 0, 0};         // {Hip, shoulder, knee}
float minLegAngles[3] = {-20, 0, 30};
float maxLegAngles[3] = {40, 90, 165};

// Effective and corrected parameters
// Lengths
float groundHeight = 0;     // mm
float legLengthHipCompensated = 0;     // mm
// Angles
float effectiveShoulderFootAngle = 0;  // rad
float shoulderFootProjectionAngle = 0;
float cosKneeAngle = 0;
float cosKneeAngleSupplementary = 0;
float sinKneeAngleSupplementary = 0;



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
    prevKinematic = millis();
    
    groundHeight = sqrt(pow(legEndPointPosition[0], 2) +
                        pow(legEndPointPosition[1], 2));
            
    effectiveShoulderFootAngle = atan2(legEndPointPosition[1],
                                       legEndPointPosition[0]);
    
    cosKneeAngle = (pow(legLengths[1], 2) + pow(legLengths[2], 2) - 
                    pow(legEndPointPosition[0], 2) - pow(legEndPointPosition[1], 2)) / 
                   (2 * legLengths[1] * legLengths[2]);
    // Knee angle
    legAngles[2] = acos(cosKneeAngle);
    
    cosKneeAngleSupplementary = - cosKneeAngle;
    sinKneeAngleSupplementary = sqrt(1 - pow(cosKneeAngleSupplementary, 2));
    
    shoulderFootProjectionAngle = atan2(legLengths[2] * sinKneeAngleSupplementary,
                                        legLengths[1] + legLengths[2] * cosKneeAngleSupplementary);

    // Shoulder angle
    legAngles[1] = effectiveShoulderFootAngle - shoulderFootProjectionAngle; 
               
    // Print values to serial (for testing)
    Serial.print("groundHeight: ");
    Serial.println(groundHeight);
    
    Serial.print("effectiveShoulderFootAngle: ");
    Serial.print(effectiveShoulderFootAngle);
    Serial.print(" ");
    Serial.println(effectiveShoulderFootAngle*180/PI);
    
    Serial.print("shoulderFootProjectionAngle: ");
    Serial.print(shoulderFootProjectionAngle);
    Serial.print(" ");
    Serial.println(shoulderFootProjectionAngle*180/PI);
    
    Serial.print("legAngles[2](Knee): ");
    Serial.print(legAngles[2]);
    Serial.print(" ");
    Serial.println(legAngles[2]*180/PI);
    
    Serial.print("legAngles[1](Shoulder): ");
    Serial.print(legAngles[1]);
    Serial.print(" ");
    Serial.println(legAngles[1]*180/PI);
        
    Serial.println("\n");
  }
}
