int flexPin = 3; //analog pin 0
int flexOut;
int flexPrev;
int flexSpeed = 200;
int lastChange = 0;
int rate = 250;
int speakerOut = 9;

void setup(){
  Serial.begin(9600);
  pinMode(speakerOut, OUTPUT);
}

void loop(){
  flexOut = analogRead(flexPin);
  flexOut = map(flexOut, 100, 400, 300, 1500);
  
  if (millis() - lastChange >= flexSpeed) {
    flexSpeed = abs(flexOut - flexPrev);
    flexPrev = flexOut;
    Serial.print("Flex speed is: ");
    Serial.println(flexSpeed);
    lastChange = millis();
  }
//  if (millis() - lastChange >= rate) {
//    Serial.print("Flex value is: ");
//    Serial.println(flexOut);
//    lastChange = millis();
//
//    int noteDuration = 1000 / random(4, 16);
//    buzz(speakerOut, flexOut, noteDuration);
//    int pauseBetweenNotes = noteDuration * 1.30;
//    delay(pauseBetweenNotes);
//    buzz(speakerOut, 0, noteDuration);
//  }
  
}

void buzz(int targetPin, long frequency, long length) {
  long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to
  //// get the total number of cycles to produce
  for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
    digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }
 
}
