#include <Servo.h>
#include <math.h>


// Timing parameters
unsigned int currentTime = 0; // ms
unsigned int prevKinematic = 0; // ms
unsigned int kinematicsRefreshTime = 10; // ms

// Leg length parameters
float upperArmLength = 150; // mm
float lowerArmLength = 150; // mm
float footOffset = 5; // mm
float upperArmLimit = 90; // degrees
float lowerArmLimit = 75; // degrees 

float legLength = 0; // mm
float legEndPointLocation[3] = {-25, 100, 0}; // mm

// Leg angle parameters
float legElbowAngle = 0; // degrees
float legShoulderAngle = 0; // degrees
float legHipAngle = 0; // degrees

void setup() {
    currentTime = millis();

    
}

void loop() {
    currentTime = millis();

    // lengEndPointLocation is updated here
    
    if ((currentTime - prevKinematic) >= kinematicsRefreshTime) {
            legLength = sqrt(pow(legEndPointLocation[0], 2) + \
                            pow(legEndPointLocation[1], 2) + \
                            pow(legEndPointLocation[2], 2));
            
            legShoulderAngle = arctan2(legEndPointLocation[0], \
                                        legEndPointLocation[1]);
            
    }
}
