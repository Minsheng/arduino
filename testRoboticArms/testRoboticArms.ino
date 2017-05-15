#include <Servo.h>
#include <NewPing.h>

/* For distance sensor */
#define ECHO_PIN 3
#define TRIG_PIN 4
#define MAX_DISTANCE 80
#define MAX_STOP_DISTANCE 10

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
unsigned int uS; // distance raw value read from sonar

unsigned int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.

Servo leftArm;
Servo rightArm;

void setup() {
  leftArm.attach(10);  // Set left servo to digital pin 10
  rightArm.attach(9);  // Set right servo to digital pin 9
  leftArm.write(180);
  rightArm.write(90);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
//  for (int i=0; i<20; i++) {
//    leftArm.write(180);
//    rightArm.write(90);
//    delay(1000);
//    leftArm.write(90);
//    rightArm.write(180);
//    delay(1000);
//  }

  // get a good sample, the larger the sample the longer it takes to read
  uS = sonar.ping_median(10);

  // convert to cm
  int cm = (int)uS / US_ROUNDTRIP_CM;

  if (cm > 0 && cm <= MAX_STOP_DISTANCE) {
    Serial.print("Ping: ");
    Serial.print(cm);
    Serial.println("cm");
    delay(5000);
    grabBall();
    delay(2000);
//    releaseBall();
  }
}

void grabBall() {
  leftArm.write(90);
  rightArm.write(180);
}

void releaseBall() {
  leftArm.write(180);
  rightArm.write(90);
}

