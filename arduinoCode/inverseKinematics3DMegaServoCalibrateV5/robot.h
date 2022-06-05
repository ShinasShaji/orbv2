#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>

class Robot {
    private:
        // Leg length parameters
        float legLengths[4] = {70,      // mm; length from hip to shoulder
                               150,     // mm; length from shoulder to knee
                               150,     // mm; length from knee to foot center
                               5        // mm; foot depth
                               };
        // Timing
        unsigned int currentTime = 0;
        unsigned int prevKinematic = 0;
        unsigned int kinematicRefreshTime = 20;
        // Serial recieve variables
        #define BAUDRATE 115200
        const byte numChars = 48;
        char receivedChars[numChars];
        boolean newData = false;
        boolean serialConnected = false;
        char testWord[] = "ping";
        unsigned int pingInterval = 500;

    public:
        // Function to establish connection through serial
        void establishSerialConnection();
        // Function to attach pins to corresponding servos
        void attachServoPins();
        //Function to detach pins from corresponding servos
        void detachServoPins();
};

#endif