#include <Servo.h>
#include <math.h>


// May not be necessary, but still
#define PI 3.1415926535897932384626433832795


// Timing
unsigned int currentTime = 0;               // ms
unsigned int prevKinematic = 0;             // ms
unsigned int kinematicsRefreshTime = 10;    // ms

// Leg length parameters
float legLengths[3] = {55,      // mm; length from hip to shoulder
                       170,     // mm; length from shoulder to knee
                       170,     // mm; length from knee to foot center
                       5        // mm; foot depth
                       };
// Note that the parameters above are currently inaccurate
float legShoulderToFootLength = 0; // mm
float legLengthHipCompensated = 0; // mm

// Leg endpoint position
float legEndPointPosition[3] = {0, 100, 55}; // mm; {front, down, outer}

// Leg angle parameters
float legAngles[3] = {0, 0, 0};         // {Hip, shoulder, knee}
float minLegAngles[3] = {-20, 0, 30};
float maxLegAngles[3] = {40, 90, 165};
float effectiveShoulderFootAngle = 0;   // rad ?


void setup() {
  currentTime = millis();

  // Setup servos, serial

}

void loop() {
  // Update loop time
  currentTime = millis();

  // Loops for serial data
  // legEndPointPosition is updated here

  if ((currentTime - prevKinematic) >= kinematicsRefreshTime) {
    legShoulderToFootLength = sqrt(pow(legEndPointLocation[0], 2) +
                                   pow(legEndPointLocation[1], 2));
            
    effectiveShoulderFootAngle = atan2(legEndPointLocation[0],
                                       legEndPointLocation[1]);
        
    // Print values to serial
    Serial.print("legShoulderToFootLength: ");
    Serial.println(legShoulderToFootLength);
        
    Serial.print("effectiveShoulderFootAngle: ");
    Serial.print(effectiveShoulderFootAngle);
    Serial.print(" ");
    Serial.println(effectiveShoulderFootAngle*180/PI);
  }
}
