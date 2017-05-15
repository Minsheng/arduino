#include <Servo.h>
Servo myServo;

int potVal = 0;
int potPin = 0;

void setup() {
  myServo.attach(3);
}

void loop() {
  potVal = analogRead(potPin);
  potVal = map(potVal, 0, 1023, 0, 360);
  myServo.write(potVal);
  delay(15);
}
