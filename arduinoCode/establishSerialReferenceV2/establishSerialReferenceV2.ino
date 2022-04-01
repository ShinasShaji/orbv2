
// Serial receive variables
#define BAUDRATE 115200
const byte numChars = 48;
char receivedChars[numChars];
boolean newData = false;
boolean serialConnected = false;
char testWord[] = "ping";

// Internal variables for receiveSerialData()
boolean recvInProgress = false;
byte ndx = 0;
char startMarker = '<';
char endMarker = '>';
char rc;


void setup() {
  
  establishSerialConnection();
  
}


void loop() {
  
}



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
      delay(500);
      
      Serial.print("<");
      Serial.print(testWord);
      Serial.println(">");
    } 
  }  
}



// Function to remove limiters from serial message
void receiveSerialData() {
  
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

