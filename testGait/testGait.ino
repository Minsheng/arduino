/*
  SweepTwoServos
  By Philip van Allen <philvanallen.com> for the VarSpeedServo.h library (October 2013)
  This example code is in the public domain
  
  Sweep two servos from 0-180, 180-0 in unison
  Uses the wait feature of the 2013 version of VarSpeedServo to start the first servo moving in the rearground
  and immediately starting a second servo moving and waiting for the second one to finish.
  
  Note that two servos will require more power than is available from the USB port - use an external power supply!
*/
#include <NewPing.h>
#include <VarSpeedServo.h> 
 
VarSpeedServo foreLeft;
VarSpeedServo foreRight;
VarSpeedServo rearLeft;
VarSpeedServo rearRight;

const int foreLeftPin = 3;
const int foreRightPin = 5;
const int rearLeftPin = 7;
const int rearRightPin = 9;

const int fullSpeed = 255;
const int fastSpeed = 127;
const int regularSpeed = 90;
const int midSpeed = 60;
const int slowSpeed = 30;

const int turtleFast = 60;
const int turtleMed = 45;
const int turtleSlow = 20;

const int maxAngle = 120;
const int minAngle = 60;

void setup() {
  foreLeft.attach(foreLeftPin);
  foreRight.attach(foreRightPin);
  rearLeft.attach(rearLeftPin);
  rearRight.attach(rearRightPin);

  // set diagonal limbs to same direction
  // fore left and rear right point to tail
  // fore right and rear left point to head
  foreLeft.write(maxAngle,fullSpeed,false);
  rearRight.write(minAngle,fullSpeed,false);
  foreRight.write(maxAngle,fullSpeed,false);
  rearLeft.write(minAngle,fullSpeed,false);
} 

void loop() {
    goForward(45, 20);
//  goRight();
//  goLeft();
//  goRear();
}

// locomotion pattern: diagonally opposite limbs move together
void goForward(int fastRate, int lowRate) {
//  servoSequencePoint foreLeftHeadSeq[] = {{100,engageSpeed},{80,disengageSpeed},{120,slowingSpeed}};
//  servoSequencePoint rearRightHeadSeq[] = {{0,engageSpeed},{80,disengageSpeed},{120,slowingSpeed}};
//  servoSequencePoint foreLeftTailSeq[] = {{40,turtleFast},{120,turtleMed},{160,turtleSlow}};
//  servoSequencePoint rearRightTailSeq[] = {{150,turtleFast},{70,turtleMed},{60,turtleSlow}};

//  servoSequencePoint foreRightHeadSeq[] = {{0,engageSpeed},{80,disengageSpeed},{120,slowingSpeed}};
//  servoSequencePoint rearLeftHeadSeq[] = {{0,engageSpeed},{80,disengageSpeed},{120,slowingSpeed}};
//  servoSequencePoint foreRightTailSeq[] = {{140,turtleFast},{90,turtleMed},{60,turtleSlow}};
//  servoSequencePoint rearLeftTailSeq[] = {{60,turtleFast},{80,turtleMed},{160,turtleSlow}};

  // fore left swing range:60
  // rear right swing range: 115
  // fore right swing range: 100
  // rear left swing range: 60
  foreLeft.write(30, fastRate, false);
  rearRight.write(160, fastRate, false);
  foreRight.write(60, lowRate, false);
  rearLeft.write(120, lowRate, true);
  
  foreRight.write(160, fastRate, false);
  rearLeft.write(20, fastRate, false);
  foreLeft.write(130, lowRate, false);
  rearRight.write(70, lowRate , true);
}

void goLeft(int fastRate, int lowRate) {
  // fore left swing range:60
  // rear right swing range: 115
  // fore right swing range: 115
  // rear left swing range: 60
  foreLeft.write(20, fastRate, false); // fore left moves forward
  rearRight.write(160, fastRate, false); // rear right moves forward
  rearLeft.write(80, lowRate, false); // rear left moves backward to lower degree
  foreRight.write(20, lowRate, true); // fore right moves backward
  
  foreRight.write(135, fastRate, false); // fore right moves forward
  rearLeft.write(20, fastRate, false); // rear left moves forward
  foreLeft.write(80, lowRate, false); // fore left moves backward to lower degree
  rearRight.write(45, lowRate, true); // rear right moves backward
}

void goRight(int fastRate, int lowRate) {
  // fore left swing range:115
  // rear right swing range: 55
  // fore right swing range: 60
  // rear left swing range: 115
  foreLeft.write(20, fastRate, false); // fore left moves forward
  rearRight.write(160, fastRate, false); // rear right moves forward
  foreRight.write(90, lowRate, false); // fore right moves backward to lower degree
  rearLeft.write(135, lowRate, true); // rear left moves backward
  
  foreRight.write(145, fastRate, false); // fore right moves forward
  rearLeft.write(20, fastRate, false); // rear left moves forward
  rearRight.write(105, lowRate, false); // rear right moves backward to lower degree
  foreLeft.write(135, lowRate, true); // fore left moves backward
}

// doesn't work well
void goRear() {
  // fore left swing range:60
  // rear right swing range: 70
  // fore right swing range: 50
  // rear left swing range: 55
  foreLeft.write(30, turtleFast, false); // forward
  rearRight.write(160, turtleFast, false); // forward
  foreRight.write(90, turtleMed, false); // backward
  rearLeft.write(90, turtleMed, true); // backward

  foreRight.write(140, turtleFast, false); // forward
  rearLeft.write(35, turtleFast, false); // forward
  foreLeft.write(90, turtleMed, false); // backward
  rearRight.write(90, turtleMed, true); // backward
}

