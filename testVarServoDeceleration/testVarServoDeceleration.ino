#include <VarSpeedServo.h>

/* Photocell simple testing sketch. 
 
Connect one end of the photocell to 5V, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground 
Connect LED from pin 11 through a resistor to ground 
For more information see http://learn.adafruit.com/photocells 

Wooden Mirror sketch created on Novemeber 7th, 2015

Iteration No.2 control 3 photoresistors + 3 servo motors (180 degree)
servo arrangement in a column
1 -> top
2 -> middle
3 -> bottom
*/

#define DEBUG 0

#define NUM_PR 3 // numbers of photoresistors
#define PR_THRESHOLD 850

#define SERVO_PIN_1 5
#define SERVO_PIN_2 4
#define SERVO_PIN_3 3
#define MAX_SPEED 255
/* push distance is rated by the proximity between the person and the frame 
0 -> initial position,
1 -> close
2 -> regular
3 -> far
*/
#define INIT_POS 0
#define PUSH_DIS_1 60
#define PUSH_DIS_2 120
#define PUSH_DIS_3 180

//int frame1PushedOut = 0;
//int frame2PushedOut = 0;
//int frame3PushedOut = 0;

int lastServoPos1 = INIT_POS;
int lastServoPos2 = INIT_POS;
int lastServoPos3 = INIT_POS;

VarSpeedServo servo1;
VarSpeedServo servo2;
VarSpeedServo servo3;

void setup(void) {
  Serial.begin(9600);
  
  servo1.attach(SERVO_PIN_1);
  servo2.attach(SERVO_PIN_2);
  servo3.attach(SERVO_PIN_3);
  
  servo1.write(INIT_POS);
  servo2.write(INIT_POS);
  servo3.write(INIT_POS);
}

void loop(void) {
  if (DEBUG) {
//    int prVal = 0;
//    for (int j=0; j<NUM_PR; j++) {
//      prVal = analogRead(j);
//      Serial.print("Sensor ");
//      Serial.print(j);
//      Serial.print("is ");
//      Serial.println(prVal);
//    }
    
    int prVal = analogRead(0);
    Serial.print("Sensor ");
    Serial.print(0);
    Serial.print("is ");
    Serial.println(prVal);
    square_motion(0, prVal);
  } else {
    int prVal = 0;
    for (int i=0; i<NUM_PR; i++) {
      prVal = analogRead(i);
      
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print("is ");
      Serial.println(prVal);
      
      square_motion(i, prVal);
    }
  }
}

void square_motion(int pr, int prVal) {
  int servoPos = compute_distance(prVal);
  switch (pr) {
    case 0:
      if (servoPos > lastServoPos1) {
        servo1.write(servoPos, MAX_SPEED, false);
      } else {
        // if last servo position is further, slowly move back
        int diff = lastServoPos1 - servoPos;
        // if last position is futher, gradually move to a closer position with lower speed
        // e.g. last pos -> 120, current pos -> 0 will yield 72, 48, 0
//        servoSequencePoint slow[] = {{lastServoPos1-diff*0.4,MAX_SPEED*0.8},
//                                    {lastServoPos1-diff*0.6,MAX_SPEED*0.4},
//                                    {servoPos,MAX_SPEED*0.2}};
//        servo1.sequencePlay(slow, 3);
        servo1.write(lastServoPos1-diff*0.4, MAX_SPEED*0.8, false);
        servo1.write(lastServoPos1-diff*0.6, MAX_SPEED*0.4, false);
        servo1.write(servoPos, MAX_SPEED*0.2, false);
      }
      
      lastServoPos1 = servoPos;
      break;
    case 1:
      if (servoPos > lastServoPos2) {
        servo2.write(servoPos, MAX_SPEED, false);
      } else {
        // if last servo position is further, slowly move back
        int diff = lastServoPos2 - servoPos;
        servoSequencePoint slow[] = {{lastServoPos2-diff*0.4,MAX_SPEED*0.8},
                                    {lastServoPos2-diff*0.6,MAX_SPEED*0.4},
                                    {servoPos,MAX_SPEED*0.2}};
        servo2.sequencePlay(slow, 3);
      }
      
      lastServoPos2 = servoPos;
      break;
    case 2:
      if (servoPos > lastServoPos3) {
        servo3.write(servoPos, MAX_SPEED, false);
      } else {
        // if last servo position is further, slowly move back
        int diff = lastServoPos3 - servoPos;
        servoSequencePoint slow[] = {{lastServoPos3-diff*0.4,MAX_SPEED*0.8},
                                    {lastServoPos3-diff*0.6,MAX_SPEED*0.4},
                                    {servoPos,MAX_SPEED*0.2}};
        servo3.sequencePlay(slow, 3);
      }
      
      lastServoPos3 = servoPos;
      break;
    default:
      break;
  }
}

int compute_distance(int prVal) {
//  if (prVal > PR_THRESHOLD*0.8 && prVal <= PR_THRESHOLD) {
//    return INIT_POS;
//  } else if (prVal > PR_THRESHOLD*0.6 && prVal <= PR_THRESHOLD*0.8) {
//    return PUSH_DIS_1;
//  } else {
//    return PUSH_DIS_3;
//  }
//  if (prVal > 400 && prVal <= 550) {
//    return INIT_POS;
//  } else if (prVal > 550 && prVal <= 650) {
//    return PUSH_DIS_1;
//  } else if (prVal > 650 && prVal <= 750) {
//    return PUSH_DIS_2;
//  } else {
//    return PUSH_DIS_3;
//  }
  if (prVal > 500 && prVal <= 800) {
    return INIT_POS;
  } else {
    return PUSH_DIS_3;
  }
}

