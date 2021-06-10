#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int upperLim = 175;
int lowerLim = 0;
int pos = upperLim;    // variable to store the servo position

int posIncrement = 10;
int delIncrement = 1;
int del = 15;
bool halt = true;

const byte numChars = 8;
char receivedChars[numChars];
bool newData = false;
bool wireless = true;

bool positionControl = false;

void setup() {
  Serial.begin (9600);
  Serial.println("Ready");
    
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(lowerLim);
  delay(5000);
  myservo.write(upperLim);
  delay(5000);
}

void loop() {
  if (positionControl) {
    recvMarked();

    if (newData){
      newData = false;
      if (receivedChars[0] == 'w') {
        pos = pos + posIncrement;
        myservo.write(pos);
      }
      else if (receivedChars[0] == 's') {
        pos = pos - posIncrement;
        myservo.write(pos);
      }
      Serial.println(pos);
    }
  }
  
  if (!positionControl) {
    recvMarked();

    if (newData){
      newData = false;
      if (receivedChars[0] == 'w') {
        del = del + delIncrement;
      }
      else if (receivedChars[0] == 's') {
        del = del - delIncrement;
      }
      else if (receivedChars[0] == 'h') {
        if (halt){
          halt = false;
        }
        else {
          halt = true;
        }
      }
      Serial.println(del);
    }
    if (!halt){
      for (pos = upperLim; pos >= lowerLim; pos -= 1) { // goes from 180 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(del);                       // waits 15ms for the servo to reach the position
      }
    
      for (pos = lowerLim; pos <= upperLim; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(del);                       // waits 15ms for the servo to reach the position
      }
    }
  }
}


void recvMarked() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = ' ';
  char endMarker = ' ';
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
        receivedChars[ndx] = '\0'; // terminate the string
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
