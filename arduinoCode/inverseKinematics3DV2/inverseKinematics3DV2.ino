#define BAUDRATE 115200
const byte numChars = 48;
char receivedChars[numChars];
boolean newData = false;
boolean serialConnected = false;
char testWord[] = "ping";


void setup() {
  
  establishSerialConnection();
  
}


void loop() {
  
}



void establishSerialConnection() {
  // Starting serial
  Serial.begin(BAUDRATE);

  while (!serialConnected) {
    receiveSerialData();
  
    if (newData) {
      newData = false;
      
      serialConnected = true;
      
      /*
      for (int i = 0; receivedChars[i]!='\0'; i++) {
        if (receivedChars[i]!=testWord[i]) {
          serialConnected = false;
          
          continue;
        }
      }
      */
      
      Serial.print("<");
      Serial.print(testWord);
      Serial.println(">");
    }     
  }
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

