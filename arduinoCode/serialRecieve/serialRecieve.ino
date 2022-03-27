/* 
Reference implementation of code to recieve messages enclosed by 
start and stop characters through a serial connection
*/


const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;


void setup() {
  Serial.begin (115200);
  Serial.println("<Ready>");
}


void loop() {
  recvMarked();

  if (newData){
    newData = false;
    Serial.print("<Recieved: ");
    Serial.print(receivedChars);
    Serial.println(">");
  }
}


void recvMarked() {
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
