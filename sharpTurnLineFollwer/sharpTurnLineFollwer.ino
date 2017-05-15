/*
** Line Follower Basic v. 0.5
** Last Update: 2013-05-21
*/

#include <Servo.h> 

// the threshold for black line
#define THRESHOLD 1010
#define DEBUG 0

/* Define motor controll inputs */
//motor A connected between A01 and A02 - Left Motor
//motor B connected between B01 and B02 - Right Motor
#define STBY 10 //standby

//Motor A
#define PWMA 3 //Speed control
#define AIN1 9 //Direction
#define AIN2 8 //Direction

//Motor B
#define PWMB 5 //Speed control
#define BIN1 11 //Direction
#define BIN2 12 //Direction

/* Define the pins for the IR receivers */
int irPins[5] = {A4, A3, A2, A1, A0};

/* Define values for the IR Sensor readings */

// an array to hold values from analogRead on the ir sensor (0-1023)
int irSensorAnalog[5] = {0,0,0,0,0};

// an array to hold boolean values (1/0) for the ir sensors, based on the analog read and the predefined THRESHOLD
int irSensorDigital[5] = {0,0,0,0,0}; 

// binary representation of the sensor reading from left to right
int irSensors = B00000;

// sensors detecting the line
//int count = 0;

// a score to determine deviation from the line [-180 ; +180]. Negative means the robot is left of the line.
int error = 0;

//  store the last value of error
int errorLast = 0;  

// a coorection value, based on the error that is used to change motor speed with PWM
int correction = 0; 

/* Set up maximum speed and speed for turning (to be used with PWM) */

// PWM to control motor speed [0 - 255]
int maxSpeed = 75;

/* variables to keep track of current speed of motors */
int motorLSpeed = 0;
int motorRSpeed = 0;

/* define ratio variables for changing the speed of motors */
int changeRatio1 = 0.1;
int changeRatio2 = 0.25;
int changeRatio3 = 0.4;
int changeRatio4 = 0.8;

void setup() {
  /* Set up motor controll pins as output */
  pinMode(STBY, OUTPUT);
  
  pinMode(AIN1,OUTPUT);        
  pinMode(AIN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  
  pinMode(BIN1,OUTPUT);        
  pinMode(BIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);

  Serial.begin(115200);
}

void loop() {
  Scan();
  UpdateError();
  UpdateCorrection();
  Drive();
}

void Scan() {
  // Initialize counters, sums etc.
 
//  count = 0;
  irSensors = B00000;
    
  for (int i = 0; i < 5; i++) {
    irSensorAnalog[i] = analogRead(irPins[i]);

    if (irSensorAnalog[i] >= THRESHOLD) {
      irSensorDigital[i] = 1;
    } else {
      irSensorDigital[i] = 0;
    }

    if (DEBUG) {
      Serial.print("A");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(irSensorAnalog[i]);
      Serial.print(" | ");
    }
    
//    count = count + irSensorDigital[i];
    int b = 4-i;
    irSensors = irSensors + (irSensorDigital[i]<<b);
  }

  if (DEBUG) {
      Serial.println();
  }
}

void UpdateError() {
  
  errorLast = error;  
  
  switch (irSensors) {
    case B00000:
      if (errorLast < 0) { error = -180;}
      else if (errorLast > 0) {error = 180;}
      Serial.println("Out of track!Crap!");
      break;
    
    case B10000: // leftmost sensor on the line
//      error = maxSpeed*changeRatio1;
      error = 120;
      Serial.println("Move left!");
      break;
    
    case B01000:
//      error = maxSpeed*changeRatio3;
      error = 60;
      Serial.println("Move slightly left!");
      break;
    
    case B00100:
      error = 0;
      Serial.println("Move forward!");
      break;

    case B00010:  
//      error = -maxSpeed*changeRatio3;
      error = -60;
      Serial.println("Move slightly right!");
      break;
      
    case B00001: // rightmost sensor on the line
//      error = -maxSpeed*changeRatio1;
      error = -120;
      Serial.println("Move right!");
      break;

    /* 2 Sensors on the line */         
     
    case B11000:
//     error = maxSpeed*changeRatio2; // turn left
      error = 150;
      break;
    
    case B01100:
//     error = maxSpeed*changeRatio4; // turn left slightly
      error = 60;
      break;
    
    case B00110: 
//     error = -maxSpeed*changeRatio4; // turn right slightly
      error = -60;
      break;
    
    case B00011: 
//     error = -maxSpeed*changeRatio2; // turn right
      error = -150;
      break;

    /* 3 Sensors on the line */    
     
//    case B11100:
//      error = 120;
//      break;
//      
//    case B01110:
//      if (errorLast > 0 && errorLast <= 60) {
//      error = 120; 
//      } else if (errorLast < 0 && errorLast >= -60) {
//      error = -120;
//      }
//      break;
//      
//    case B00111:
//     error = -120;
//     break;

    /* 4 Sensors on the line */       
//    case B11110:
//     error = -150;
//     break;
//    
//    case B01111:
//     error = 150;
//     break;

   /* 5 Sensors on the line */
//    case B11111:
//     break;
       
    default:
    error = errorLast;
  }
}

void UpdateCorrection() {
//  if (error >= 0 && error < 30) {
//    correction = 15;
//  } else if (error >=30 && error < 60) {
//    correction = 40;
//  } else if (error >=60 && error < 90) {
//    correction = 55;
//  } else if (error >=120 && error < 150) {
//    correction = 75;
//  } else if (error >=150 && error < 180) {
//    correction = 255;
//  } else if (error >=180) {
//    correction = 305;
//  }
//  
//  if (error <= 0 && error > -30) {
//    correction = -15;
//  } else if (error <= -30 && error > -60) {
//    correction = -40;
//  } else if (error <= -60 && error > -90) {
//    correction = -55;
//  } else if (error <= -120 && error > -150) {
//    correction = -75;
//  } else if (error <= -150 && error > -180) {
//    correction = -255;
//  } else if (error <= -180) {
//    correction = -305;
//  }
  
  
//  if (correction >= 0) {
//    motorRSpeed = maxSpeed - correction;
//    motorLSpeed = maxSpeed;
//  }
//  
//  else if (correction < 0) {
//    motorRSpeed = maxSpeed;
//    motorLSpeed = maxSpeed + correction;
//  }

  // use the error value directly as the correction value
  correction = error;
  if (correction >= 0) {
    // map correction value to a smaller speed value so the robot moves slower
    int adjust = map(correction, 0, 305, 0, maxSpeed);
    motorLSpeed = maxSpeed - adjust;
    motorRSpeed = maxSpeed;
  } else if (correction < 0) {
    // map correction value to a smaller speed value so the robot moves slower
    int adjust = map(correction, -305, 0, -maxSpeed, 0);
    motorLSpeed = maxSpeed;
    motorRSpeed = maxSpeed + adjust;
  }

//  Serial.print("Left motor speed: ");
//  Serial.println(motorLSpeed);
//  Serial.print("Right motor speed: ");
//  Serial.println(motorRSpeed);
//  Serial.println("--------------------");
}

void Drive() {
  // reset the right motor speed to max speed of 255/-255
  if (motorRSpeed > 255) {
    motorRSpeed = 255;
  } else if (motorRSpeed < -255) {
    motorRSpeed = -255;
  }

  // reset the left motor speed to max speed of 255/-255
  if (motorLSpeed > 255) {
    motorLSpeed = 255;
  } else if (motorLSpeed < -255) {
    motorLSpeed = -255;
  }

  // disable standby
  digitalWrite(STBY, HIGH);
  
  if (motorRSpeed > 0) { // right motor forward (using PWM)
     analogWrite(PWMB, motorRSpeed);
     digitalWrite(BIN1, HIGH);
     digitalWrite(BIN2, LOW);
  } else if (motorRSpeed < 0) { // right motor reverse (using PWM)
     analogWrite(PWMB, abs(motorRSpeed));
     digitalWrite(BIN1, LOW);
     digitalWrite(BIN2, HIGH);
  } else if (motorRSpeed == 0) { // right motor fast stop
     digitalWrite(PWMB, HIGH);
     digitalWrite(BIN1, LOW);
     digitalWrite(BIN2, LOW);
  }
  
  if (motorLSpeed > 0) { // left motor forward (using PWM)
     analogWrite(PWMA, motorLSpeed);
     digitalWrite(AIN1, HIGH);
     digitalWrite(AIN2, LOW);
  } else if (motorLSpeed < 0) { // left motor reverse (using PWM)
     analogWrite(PWMA, abs(motorLSpeed));
     digitalWrite(AIN1, LOW);
     digitalWrite(AIN2, HIGH);
  } else if (motorLSpeed == 0) { // left motor fast stop
     digitalWrite(PWMA, HIGH);
     digitalWrite(AIN1, LOW);
     digitalWrite(AIN2, LOW);
  }
}
