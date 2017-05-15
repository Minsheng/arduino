/*
Easy Line Follower Robot -DIYhacking.com Arvind Sanjeev

Quick and easy line following robot using an IR reflectance
array.Connect Vcc and Gnd to the Pololu QTR-8A sensor from 
the arduino. Connect the pins 1,2,3,... to arduino's analog
pins 0,1,2,3,4,5.
If the average of the 3 values of the sensors on the left is
greater than the average of those on the right, then the robot
moves left and vice versa.
NOTE : The values in the code for analog voltages would have 
to be modified if you are not using the Pololu QTR-8A reflectance
array sensor. Use trial and error to find out max and min 
values for your own IR array sensor.
 
 */
#include <Servo.h> 

int mid = 0;
int mn = 0;
int mx = 0;

void setup() {
  Serial.begin(115200);
  
//  for (int i=0; i<5000; i++) {
//    
//    int val = 0;
//
//    //Calibrating the sensor, finding max and min reflectance values.
//    for(int j=0; j<=5; j++) {
//      val = analogRead(j);  
//      if(val >= mx)
//        mx = val;
//    
//      if(val <= mn)
//        mn = val;
//    }
//    delay(1);
//  }
  
//  mid = ((mx + mn)/2);
}
  
void loop() {
  int s0 = 0;
  int s1 = 0;
  int s2 = 0;
  int s3 = 0;
  int s4 = 0;
  int s5 = 0;
  
  s0 = analogRead(0);//Signal pin 1 on the board
  s1 = analogRead(1);//Signal pin 2 on the board
  s2 = analogRead(2);//Signal pin 3 on the board
  s3 = analogRead(3);//Signal pin 4 on the board
  s4 = analogRead(4);//Signal pin 5 on the board
  s5 = analogRead(5);//Signal pin 6 on the board
  
//  Serial.print("Mid: ");
//  Serial.print(mid); 
  Serial.print(" ");
  Serial.print(s0); 
  Serial.print(" ");
  Serial.print(s1); 
  Serial.print(" ");
  Serial.print(s2); 
  Serial.print(" ");
  Serial.print(s3); 
  Serial.print(" ");
  Serial.print(s4); 
  Serial.print(" ");
  Serial.print(s5); 
  Serial.print(" ");
  Serial.println();
  
//  //Move right
//  if ((((s0+s1+s2)/3)>(((s3+s4+s5)/3)+240))) {
//    turnRight();
//    Serial.print(" RIGHT");
//    delay(abs((((s5+s4+s3)/3)-((s0+s1+s2)/3))/2));
//  }
//
//  //Move left
//  if ((((s0+s1+s2)/3)<(((s3+s4+s5)/3)-240))) {
//    turnLeft();
//    Serial.print(" LEFT");
//    delay(abs((((s5+s4+s3)/3)-((s0+s1+s2)/3))/2));
//  }

//  //Stop if all the sensors give low reflectance values
//  if ((s0 > mid)&&(s5 > mid)) {
//    stop();
//    Serial.print(" STOP"); 
//    
//    delay(1000);
//  }
}

//void turnRight() {
//  move(1, SPEED, 0); 
//  move(2, SPEED, 1);
//}
//
//void turnLeft() {
//  move(1, SPEED, 1); 
//  move(2, SPEED, 0);
//}
//
//void moveForward() {
//  move(1, SPEED, 0); 
//  move(2, SPEED, 0);
//}
//
//void moveBackward() {
//  move(1, 100, 1); 
//  move(2, 100, 1);
//}
//
//void move(int motor, int speed, int direction) {
//  //Move specific motor at speed and direction
//  //motor: 0 for B 1 for A
//  //speed: 0 is off, and 255 is full speed
//  //direction: 0 clockwise, 1 counter-clockwise
//
//  digitalWrite(STBY, HIGH); //disable standby
//
//  boolean inPin1 = LOW;
//  boolean inPin2 = HIGH;
//
//  if (direction == 1) {
//    inPin1 = HIGH;
//    inPin2 = LOW;
//  }
//
//  if (motor == 1) {
//    digitalWrite(AIN1, inPin1);
//    digitalWrite(AIN2, inPin2);
//    analogWrite(PWMA, speed);
//  } else {
//    digitalWrite(BIN1, inPin1);
//    digitalWrite(BIN2, inPin2);
//    analogWrite(PWMB, speed);
//  }
//}
//
//void stop(){
////enable standby  
//  digitalWrite(STBY, LOW); 
//}
