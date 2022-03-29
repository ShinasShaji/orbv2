
#define echoPin 2
#define trigPin 3

long duration;
int distance;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
}

void loop() {
  // Clear the trigPin :
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  // Trigger the sensor :
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  //read the distance:
  duration=pulseIn(echoPin,HIGH);
  distance=(duration*0.034/2);  //speed of sound = 0.034 centimeter/microsecond

  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.println(" cm ");

  //to convert to inches :
  //inches = microsecondsToInches(duration);
  
  delay(1000);


}
