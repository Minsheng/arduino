#include <Servo.h> 

/*
 *This example shows how to set up a repetitive servo sweep without using loops/delays 
 *This base code can be extended to support multiple servos moving at different rates
 *The next steps with this coded would be to link sensor input to one of the 3 main variables:
 *endAngle
 *startAngle
 *moveRate
 *
 */


#define DEBUG 1

Servo servo1;  // create servo object to control a servo 
int servoPin = 8;  // the pin servo1 is attached to

int START_ANGLE_PIN = 0;
int END_ANGLE_PIN = 1;
int SPEED_PIN = 2;
//int RAND_PIN = 3;

int startAngle = 0;//bottom limit of the sweep
int endAngle = 0;//top limit of the sweep

int target = 0;
int angleRx = 0;    // variable to store the servo angleRxition

long lastMove;
int moveRate = 60;
 
void setup() 
{ 
  servo1.attach(servoPin);  // attaches the servo on pin 9 to the servo object 
  servo1.write(endAngle);  //set the sweep to the min side
  
  Serial.begin(9600);
}
 
void loop() 
{

  int startAnglePotVal = analogRead(START_ANGLE_PIN);
  int endAnglePotVal = analogRead(END_ANGLE_PIN);
  int speedPotVal = analogRead(SPEED_PIN);

  if (DEBUG) {
    Serial.println("Printing raw values...");
    Serial.print(startAnglePotVal);
    Serial.print(" | ");
    Serial.print(endAnglePotVal);
    Serial.print(" | ");
    Serial.print(speedPotVal);
    Serial.println();
  }
  
  // map the potentiometer values
  startAngle = map(startAnglePotVal, 0, 1023, 0, 90);
  endAngle = map(endAnglePotVal, 0, 1023, 90, 180);
  moveRate = map(speedPotVal, 0, 1023, 0, 120);

  if (DEBUG) {
    Serial.println("Printing mapped values...");
    Serial.print(startAngle);
    Serial.print(" | ");
    Serial.print(endAngle);
    Serial.print(" | ");
    Serial.print(moveRate);
    Serial.println();
  }
  
  // set the target to the end angle
  target = endAngle;

 if((millis()-lastMove)>=moveRate)
{
  
   if(target==startAngle)
  { 
       if((angleRx+1)<=startAngle)
       {
       angleRx++;  //add 1 to the variable it is the shorthand for writing:  angleRx = angleRx+1;
       lastMove=millis();
       }
         else
         {
         target=endAngle;
         }
   
   }
   if(target==endAngle)
  { 
       if((angleRx-1)>=endAngle)
       {
       angleRx--;  //subtract 1 from the variable it is the shorthand for writing:  angleRx = angleRx-1;
       lastMove=millis();
       }
         else
         {
         target=startAngle;
         }
   
   }
} 
 

servo1.write(angleRx);
} 
